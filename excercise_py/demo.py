"""
================================================================================
WAREHOUSE FIFO MANAGEMENT SYSTEM - SIMPLE VERSION
================================================================================

Logic FIFO:
- Mỗi position = 1 item (không có quantity)
- NHẬP: P3 → P2 → P1 → P0 (liền kề)
- XUẤT: P3 → P2 → P1 → P0 (FIFO)
- Hàng luôn liên tục, không có lỗ trống

Author: Warehouse Team
Date: 2024
================================================================================
"""

from pymongo import MongoClient, ASCENDING
from datetime import datetime
from typing import Optional, Dict, List
import uuid
from enum import Enum

# ================================================================================
# ENUMS & CONSTANTS
# ================================================================================


class RackStatus(Enum):
    """Trạng thái của rack"""

    EMPTY = "empty"  # Trống hoàn toàn
    PARTIAL = "partial"  # Có hàng nhưng chưa đầy
    FULL = "full"  # Đầy


class OrderStatus(Enum):
    """Trạng thái đơn hàng"""

    PENDING = "pending"  # Chờ xử lý
    IN_PROGRESS = "in_progress"  # Đang xử lý
    COMPLETED = "completed"  # Hoàn thành
    CANCELLED = "cancelled"  # Đã hủy


class TransactionType(Enum):
    """Loại giao dịch"""

    INBOUND = "inbound"  # Nhập hàng
    OUTBOUND = "outbound"  # Xuất hàng


# ================================================================================
# WAREHOUSE CONTROLLER CLASS
# ================================================================================


class WarehouseControllerFIFO:
    """
    Warehouse Controller với logic FIFO đơn giản
    Mỗi position = 1 item (không có quantity)
    """

    def __init__(self, mongo_uri: str, db_name: str):
        """Khởi tạo kết nối MongoDB"""
        self.client = MongoClient(mongo_uri)
        self.db = self.client[db_name]

        # Collections
        self.racks = self.db.racks
        self.inbound_orders = self.db.inbound_orders
        self.outbound_orders = self.db.outbound_orders
        self.transactions = self.db.transactions

        # Tạo indexes
        self._create_indexes()

    def _create_indexes(self):
        """Tạo indexes cho performance"""
        self.racks.create_index([("rack_code", ASCENDING)], unique=True)
        self.racks.create_index([("product_id", ASCENDING)])
        self.inbound_orders.create_index([("order_id", ASCENDING)], unique=True)
        self.outbound_orders.create_index([("order_id", ASCENDING)], unique=True)
        self.transactions.create_index([("timestamp", ASCENDING)])

    # ============================================================================
    # QUẢN LÝ RACK
    # ============================================================================

    def create_rack(
        self, rack_code: str, capacity: int = 4, location: Dict = None
    ) -> Dict:
        """
        Tạo rack mới với positions cố định

        Args:
            rack_code: Mã rack (VD: "RACK-A-001")
            capacity: Số lượng positions (mặc định 4)
            location: Vị trí vật lý (zone, aisle, level)

        Returns:
            Dict: Thông tin rack đã tạo
        """
        # Kiểm tra rack đã tồn tại chưa
        if self.racks.find_one({"rack_code": rack_code}):
            raise ValueError(f"❌ Rack {rack_code} đã tồn tại")

        # Tạo positions cố định
        positions = []
        for i in range(capacity):
            positions.append(
                {
                    "position_index": i,
                    "slot_id": f"{rack_code}-P{i}",
                    "status": "empty",
                    "item": None,
                }
            )

        # Tạo rack document
        rack = {
            "rack_code": rack_code,
            "location": location or {},
            "capacity": capacity,
            "occupied_count": 0,
            "available_count": capacity,
            "product_id": None,
            "status": RackStatus.EMPTY.value,
            "positions": positions,
            "created_at": datetime.utcnow(),
            "updated_at": datetime.utcnow(),
        }

        self.racks.insert_one(rack)
        print(f"✅ Tạo rack: {rack_code}")
        return rack

    def get_rack_info(self, rack_code: str) -> Optional[Dict]:
        """Lấy thông tin rack"""
        return self.racks.find_one({"rack_code": rack_code})

    def find_inbound_position(self, rack: Dict) -> Optional[int]:
        """
        Tìm vị trí để NHẬP hàng

        Logic:
        - Tìm vị trí có hàng gần FRONT nhất (index nhỏ nhất)
        - Nhập vào vị trí LIỀN KỀ phía trước (index - 1)
        - Nếu rack trống: nhập vào P3 (BACK)

        Returns:
            Position index hoặc None nếu rack đầy
        """
        # Tìm position có hàng gần FRONT nhất
        front_most_occupied = None
        for i in range(rack["capacity"]):  # 0, 1, 2, 3
            if rack["positions"][i]["status"] == "occupied":
                front_most_occupied = i
                break

        if front_most_occupied is None:
            # Rack trống → nhập vào P3 (BACK)
            return rack["capacity"] - 1

        # Có hàng → nhập vào vị trí liền kề phía trước
        next_position = front_most_occupied - 1

        if next_position < 0:
            # Rack đầy
            return None

        return next_position

    def find_outbound_position(self, rack: Dict) -> Optional[int]:
        """
        Tìm vị trí để XUẤT hàng

        Logic:
        - Tìm từ P3 → P2 → P1 → P0
        - Lấy vị trí CÓ HÀNG đầu tiên

        Returns:
            Position index hoặc None nếu rack trống
        """
        for i in range(rack["capacity"] - 1, -1, -1):  # 3, 2, 1, 0
            if rack["positions"][i]["status"] == "occupied":
                return i

        return None

    # ============================================================================
    # NHẬP HÀNG (INBOUND)
    # ============================================================================

    def create_inbound_order(
        self,
        product_id: str,
        total_items: int,
        batch_number: str,
        supplier: str,
        **kwargs,
    ) -> Dict:
        """
        Tạo đơn nhập hàng

        Args:
            product_id: Mã sản phẩm
            total_items: Tổng số items cần nhập (số positions)
            batch_number: Số lô hàng
            supplier: Nhà cung cấp

        Returns:
            Dict: Thông tin đơn nhập hàng
        """
        order_id = (
            f"IN-{datetime.now().strftime('%Y%m%d')}-{uuid.uuid4().hex[:6].upper()}"
        )

        inbound_order = {
            "order_id": order_id,
            "product_id": product_id,
            "total_items": total_items,
            "received_items": 0,
            "batch_number": batch_number,
            "supplier": supplier,
            "status": OrderStatus.PENDING.value,
            "items": [],  # Danh sách items đã nhập vào rack
            "metadata": kwargs.get("metadata", {}),
            "created_by": kwargs.get("operator", "system"),
            "created_at": datetime.utcnow(),
            "updated_at": datetime.utcnow(),
            "completed_at": None,
        }

        self.inbound_orders.insert_one(inbound_order)
        print(f"✅ Tạo đơn nhập: {order_id} ({total_items} items)")
        return inbound_order

    def add_item_to_rack(
        self, rack_code: str, product_id: str, batch_number: str, **kwargs
    ) -> Dict:
        """
        Thêm 1 item vào rack (1 position)

        Args:
            rack_code: Mã rack
            product_id: Mã sản phẩm
            batch_number: Số lô hàng

        Returns:
            Dict: Thông tin item đã thêm
        """
        rack = self.racks.find_one({"rack_code": rack_code})

        if not rack:
            raise ValueError(f"❌ Rack {rack_code} không tồn tại")

        if rack["available_count"] == 0:
            raise ValueError(f"❌ Rack {rack_code} đã đầy")

        if rack["product_id"] and rack["product_id"] != product_id:
            raise ValueError(
                f"❌ Rack {rack_code} đang chứa sản phẩm khác: {rack['product_id']}"
            )

        # Tìm vị trí nhập (FIFO)
        position_index = self.find_inbound_position(rack)
        if position_index is None:
            raise ValueError("❌ Không tìm thấy vị trí để nhập")

        # Tạo item mới
        item_id = f"ITEM-{uuid.uuid4().hex[:8].upper()}"

        new_item = {
            "item_id": item_id,
            "product_id": product_id,
            "batch_number": batch_number,
            "entry_date": datetime.utcnow(),
            "expiry_date": kwargs.get("expiry_date"),
            "inbound_order_id": kwargs.get("inbound_order_id"),
            "metadata": kwargs.get("metadata", {}),
        }

        # Tính toán trạng thái mới
        new_occupied = rack["occupied_count"] + 1
        new_available = rack["available_count"] - 1

        if new_available == 0:
            new_status = RackStatus.FULL.value
        else:
            new_status = RackStatus.PARTIAL.value

        # Update rack
        self.racks.update_one(
            {"rack_code": rack_code},
            {
                "$set": {
                    f"positions.{position_index}.status": "occupied",
                    f"positions.{position_index}.item": new_item,
                    "product_id": product_id,
                    "status": new_status,
                    "occupied_count": new_occupied,
                    "available_count": new_available,
                    "updated_at": datetime.utcnow(),
                }
            },
        )

        # Ghi transaction log
        self._log_transaction(
            transaction_type=TransactionType.INBOUND.value,
            rack_code=rack_code,
            product_id=product_id,
            item_id=item_id,
            batch_number=batch_number,
            position=position_index,
            operator=kwargs.get("operator", "system"),
        )

        return {"item_id": item_id, "position_index": position_index}

    def process_inbound(
        self, order_id: str, rack_code: str, operator: str = "system"
    ) -> Dict:
        """
        Xử lý nhập 1 item vào rack theo đơn

        Args:
            order_id: Mã đơn nhập hàng
            rack_code: Mã rack
            operator: Người thực hiện

        Returns:
            Dict: Kết quả nhập hàng
        """
        order = self.inbound_orders.find_one({"order_id": order_id})
        if not order:
            raise ValueError(f"❌ Đơn nhập {order_id} không tồn tại")

        if order["status"] == OrderStatus.COMPLETED.value:
            raise ValueError(f"❌ Đơn nhập {order_id} đã hoàn thành")

        # Kiểm tra số lượng
        remaining = order["total_items"] - order["received_items"]
        if remaining <= 0:
            raise ValueError(f"❌ Đơn hàng đã nhập đủ")

        # Thêm vào rack
        result = self.add_item_to_rack(
            rack_code=rack_code,
            product_id=order["product_id"],
            batch_number=order["batch_number"],
            inbound_order_id=order_id,
            operator=operator,
            metadata={"supplier": order["supplier"]},
        )

        # Update đơn hàng
        new_received = order["received_items"] + 1
        new_status = (
            OrderStatus.COMPLETED.value
            if new_received >= order["total_items"]
            else OrderStatus.IN_PROGRESS.value
        )

        self.inbound_orders.update_one(
            {"order_id": order_id},
            {
                "$set": {
                    "received_items": new_received,
                    "status": new_status,
                    "updated_at": datetime.utcnow(),
                    "completed_at": (
                        datetime.utcnow()
                        if new_status == OrderStatus.COMPLETED.value
                        else None
                    ),
                },
                "$push": {
                    "items": {
                        "item_id": result["item_id"],
                        "rack_code": rack_code,
                        "position": result["position_index"],
                        "timestamp": datetime.utcnow(),
                    }
                },
            },
        )

        return {
            "order_id": order_id,
            "item_id": result["item_id"],
            "rack_code": rack_code,
            "position": result["position_index"],
            "received_items": new_received,
            "total_items": order["total_items"],
            "status": new_status,
        }

    def get_inbound_order(self, order_id: str) -> Optional[Dict]:
        """Lấy thông tin đơn nhập hàng"""
        return self.inbound_orders.find_one({"order_id": order_id})

    # ============================================================================
    # XUẤT HÀNG (OUTBOUND)
    # ============================================================================

    def create_outbound_order(
        self, product_id: str, required_items: int, customer: str, **kwargs
    ) -> Dict:
        """
        Tạo đơn xuất hàng

        Args:
            product_id: Mã sản phẩm
            required_items: Số items cần xuất (số positions)
            customer: Khách hàng

        Returns:
            Dict: Thông tin đơn xuất hàng
        """
        # Kiểm tra tồn kho
        available = self.get_available_items(product_id)
        if available < required_items:
            raise ValueError(
                f"❌ Không đủ tồn kho. Yêu cầu: {required_items}, "
                f"Có sẵn: {available}"
            )

        order_id = (
            f"OUT-{datetime.now().strftime('%Y%m%d')}-{uuid.uuid4().hex[:6].upper()}"
        )

        outbound_order = {
            "order_id": order_id,
            "product_id": product_id,
            "required_items": required_items,
            "picked_items": 0,
            "customer": customer,
            "status": OrderStatus.PENDING.value,
            "items": [],  # Danh sách items đã xuất
            "metadata": kwargs.get("metadata", {}),
            "created_by": kwargs.get("operator", "system"),
            "created_at": datetime.utcnow(),
            "updated_at": datetime.utcnow(),
            "completed_at": None,
        }

        self.outbound_orders.insert_one(outbound_order)
        print(f"✅ Tạo đơn xuất: {order_id} ({required_items} items)")
        return outbound_order

    def remove_item_from_rack(self, rack_code: str, operator: str = "system") -> Dict:
        """
        Lấy 1 item từ rack (1 position)

        Args:
            rack_code: Mã rack
            operator: Người thực hiện

        Returns:
            Dict: Thông tin item đã lấy
        """
        rack = self.racks.find_one({"rack_code": rack_code})

        if not rack:
            raise ValueError(f"❌ Rack {rack_code} không tồn tại")

        if rack["occupied_count"] == 0:
            raise ValueError(f"❌ Rack {rack_code} đang trống")

        # Tìm vị trí xuất (FIFO)
        position_index = self.find_outbound_position(rack)
        if position_index is None:
            raise ValueError("❌ Không tìm thấy item để xuất")

        # Lấy item ra
        item = rack["positions"][position_index]["item"]

        # Tính toán trạng thái mới
        new_occupied = rack["occupied_count"] - 1
        new_available = rack["available_count"] + 1

        if new_occupied == 0:
            new_status = RackStatus.EMPTY.value
            new_product_id = None
        else:
            new_status = RackStatus.PARTIAL.value
            new_product_id = rack["product_id"]

        # Update rack
        self.racks.update_one(
            {"rack_code": rack_code},
            {
                "$set": {
                    f"positions.{position_index}.status": "empty",
                    f"positions.{position_index}.item": None,
                    "product_id": new_product_id,
                    "status": new_status,
                    "occupied_count": new_occupied,
                    "available_count": new_available,
                    "updated_at": datetime.utcnow(),
                }
            },
        )

        # Ghi transaction log
        self._log_transaction(
            transaction_type=TransactionType.OUTBOUND.value,
            rack_code=rack_code,
            product_id=item["product_id"],
            item_id=item["item_id"],
            batch_number=item["batch_number"],
            position=position_index,
            operator=operator,
        )

        return {"item": item, "position_index": position_index}

    def process_outbound(
        self, order_id: str, rack_code: str, operator: str = "system"
    ) -> Dict:
        """
        Xử lý xuất 1 item từ rack theo đơn

        Args:
            order_id: Mã đơn xuất hàng
            rack_code: Mã rack
            operator: Người thực hiện

        Returns:
            Dict: Kết quả xuất hàng
        """
        order = self.outbound_orders.find_one({"order_id": order_id})
        if not order:
            raise ValueError(f"❌ Đơn xuất {order_id} không tồn tại")

        if order["status"] == OrderStatus.COMPLETED.value:
            raise ValueError(f"❌ Đơn xuất {order_id} đã hoàn thành")

        # Lấy hàng từ rack
        result = self.remove_item_from_rack(rack_code, operator)
        item = result["item"]

        # Update đơn hàng
        new_picked = order["picked_items"] + 1
        new_status = (
            OrderStatus.COMPLETED.value
            if new_picked >= order["required_items"]
            else OrderStatus.IN_PROGRESS.value
        )

        self.outbound_orders.update_one(
            {"order_id": order_id},
            {
                "$set": {
                    "picked_items": new_picked,
                    "status": new_status,
                    "updated_at": datetime.utcnow(),
                    "completed_at": (
                        datetime.utcnow()
                        if new_status == OrderStatus.COMPLETED.value
                        else None
                    ),
                },
                "$push": {
                    "items": {
                        "item_id": item["item_id"],
                        "rack_code": rack_code,
                        "batch_number": item["batch_number"],
                        "position": result["position_index"],
                        "timestamp": datetime.utcnow(),
                    }
                },
            },
        )

        return {
            "order_id": order_id,
            "item_id": item["item_id"],
            "rack_code": rack_code,
            "position": result["position_index"],
            "picked_items": new_picked,
            "required_items": order["required_items"],
            "status": new_status,
        }

    def get_outbound_order(self, order_id: str) -> Optional[Dict]:
        """Lấy thông tin đơn xuất hàng"""
        return self.outbound_orders.find_one({"order_id": order_id})

    # ============================================================================
    # TIỆN ÍCH & BÁO CÁO
    # ============================================================================

    def get_available_items(self, product_id: str) -> int:
        """
        Lấy tổng số items có sẵn của sản phẩm

        Args:
            product_id: Mã sản phẩm

        Returns:
            int: Tổng số items (positions)
        """
        pipeline = [
            {"$match": {"product_id": product_id}},
            {"$group": {"_id": "$product_id", "total": {"$sum": "$occupied_count"}}},
        ]

        result = list(self.racks.aggregate(pipeline))
        return result[0]["total"] if result else 0

    def get_product_inventory(self, product_id: str) -> Dict:
        """
        Lấy thông tin tồn kho chi tiết của sản phẩm

        Args:
            product_id: Mã sản phẩm

        Returns:
            Dict: Thông tin tồn kho
        """
        racks = self.racks.find({"product_id": product_id})

        inventory = {"product_id": product_id, "total_items": 0, "racks": []}

        for rack in racks:
            rack_info = {
                "rack_code": rack["rack_code"],
                "location": rack.get("location", {}),
                "occupied_count": rack["occupied_count"],
                "items": [],
            }

            for pos in rack["positions"]:
                if pos["status"] == "occupied" and pos["item"]:
                    item = pos["item"]
                    inventory["total_items"] += 1

                    rack_info["items"].append(
                        {
                            "position": pos["position_index"],
                            "item_id": item["item_id"],
                            "batch_number": item["batch_number"],
                            "entry_date": item["entry_date"],
                        }
                    )

            if rack_info["items"]:
                inventory["racks"].append(rack_info)

        return inventory

    def _log_transaction(self, **kwargs):
        """Ghi log transaction"""
        transaction = {
            "transaction_id": f"TXN-{uuid.uuid4().hex[:8].upper()}",
            "transaction_type": kwargs.get("transaction_type"),
            "rack_code": kwargs.get("rack_code"),
            "product_id": kwargs.get("product_id"),
            "item_id": kwargs.get("item_id"),
            "batch_number": kwargs.get("batch_number"),
            "position": kwargs.get("position"),
            "operator": kwargs.get("operator"),
            "timestamp": datetime.utcnow(),
        }

        self.transactions.insert_one(transaction)

    # ============================================================================
    # HIỂN THỊ VISUAL
    # ============================================================================

    def display_rack(self, rack_code: str):
        """Hiển thị rack dạng visual"""
        rack = self.racks.find_one({"rack_code": rack_code})
        if not rack:
            print(f"❌ Rack {rack_code} không tồn tại")
            return

        # Header
        print(f"\n{'═'*70}")
        print(
            f"  RACK: {rack['rack_code']} | {rack['status'].upper():8} | "
            f"Occupied: {rack['occupied_count']}/{rack['capacity']}"
        )
        print(f"{'═'*70}")
        print("\n  ← FRONT (Xuất sau)                    BACK (Xuất trước) →\n")

        print("  ┌──────────────┬──────────────┬──────────────┬──────────────┐")

        # Position index
        line = "  │"
        for pos in rack["positions"]:
            line += f"      P{pos['position_index']}      │"
        print(line)

        # Status
        line = "  │"
        for pos in rack["positions"]:
            if pos["status"] == "occupied":
                line += f"   OCCUPIED   │"
            else:
                line += f"    EMPTY     │"
        print(line)

        # Item ID
        line = "  │"
        for pos in rack["positions"]:
            if pos["item"]:
                short_id = pos["item"]["item_id"][-6:]
                line += f"   {short_id:^8}  │"
            else:
                line += f"      --      │"
        print(line)

        print("  └──────────────┴──────────────┴──────────────┴──────────────┘")

        # Arrows (IN/OUT indicators)
        next_in = self.find_inbound_position(rack)
        next_out = self.find_outbound_position(rack)

        arrow = "  "
        for i in range(rack["capacity"]):
            if i == next_in and rack["available_count"] > 0:
                arrow += "     ↓ IN      "
            elif i == next_out and rack["occupied_count"] > 0:
                arrow += "     ↑ OUT     "
            else:
                arrow += "               "
        print(arrow)

        # Chi tiết items
        occupied_items = [p for p in rack["positions"] if p["status"] == "occupied"]
        if occupied_items:
            print(f"\n  📦 Chi tiết ({len(occupied_items)} items):")
            for pos in occupied_items:
                item = pos["item"]
                print(
                    f"     P{pos['position_index']}: {item['item_id']} | "
                    f"Batch: {item['batch_number']}"
                )

        print()


# ================================================================================
# DEMO & TESTING
# ================================================================================


def demo_complete_workflow():
    """Demo quy trình hoàn chỉnh"""
    # Kết nối MongoDB
    wc = WarehouseControllerFIFO(
        mongo_uri="mongodb://localhost:27017/", db_name="warehouse_fifo_simple"
    )

    # Reset database
    wc.racks.delete_many({})
    wc.inbound_orders.delete_many({})
    wc.outbound_orders.delete_many({})
    wc.transactions.delete_many({})

    print("\n" + "=" * 70)
    print("  DEMO: WAREHOUSE FIFO - SIMPLE VERSION")
    print("  (Mỗi position = 1 item, không có quantity)")
    print("=" * 70)

    # ========== BƯỚC 1: Tạo Rack ==========
    print("\n" + "─" * 70)
    print("BƯỚC 1: TẠO RACK")
    print("─" * 70)

    wc.create_rack(
        rack_code="RACK-A-001",
        capacity=4,
        location={"zone": "A", "aisle": "01", "level": 1},
    )
    wc.display_rack("RACK-A-001")

    # ========== BƯỚC 2: Nhập hàng ==========
    print("\n" + "─" * 70)
    print("BƯỚC 2-4: NHẬP HÀNG (INBOUND)")
    print("─" * 70)

    # Tạo đơn nhập 4 items
    inbound = wc.create_inbound_order(
        product_id="PROD-001",
        total_items=4,
        batch_number="BATCH-2024-001",
        supplier="Supplier ABC",
        operator="user123",
    )

    # Nhập từng item vào rack
    print(f"\n  📦 Nhập 3 items vào RACK-A-001:")

    for i in range(3):
        result = wc.process_inbound(
            order_id=inbound["order_id"], rack_code="RACK-A-001", operator="user123"
        )
        print(
            f"     {i+1}. Nhập item vào P{result['position']} | "
            f"Item: {result['item_id']} | "
            f"Progress: {result['received_items']}/{result['total_items']}"
        )

    wc.display_rack("RACK-A-001")

    # ========== BƯỚC 3: Kiểm tra tồn kho ==========
    print("\n" + "─" * 70)
    print("BƯỚC 5: KIỂM TRA TỒN KHO")
    print("─" * 70)

    inventory = wc.get_product_inventory("PROD-001")
    print(f"\n  📊 Tồn kho PROD-001:")
    print(f"     • Tổng số items: {inventory['total_items']}")
    print(f"     • Số rack: {len(inventory['racks'])}")

    # ========== BƯỚC 4: Xuất hàng ==========
    print("\n" + "─" * 70)
    print("BƯỚC 6-7: XUẤT HÀNG (OUTBOUND - FIFO)")
    print("─" * 70)

    # Tạo đơn xuất 2 items
    outbound = wc.create_outbound_order(
        product_id="PROD-001",
        required_items=2,
        customer="Customer XYZ",
        operator="user456",
    )

    # Xuất 2 items
    print(f"\n  📤 Xuất 2 items từ RACK-A-001 (FIFO - lấy từ P3 trước):")

    for i in range(2):
        result = wc.process_outbound(
            order_id=outbound["order_id"], rack_code="RACK-A-001", operator="user456"
        )
        print(
            f"     {i+1}. Xuất item từ P{result['position']} | "
            f"Item: {result['item_id']} | "
            f"Progress: {result['picked_items']}/{result['required_items']}"
        )

    wc.display_rack("RACK-A-001")

    # ========== BƯỚC 5: Nhập thêm hàng mới ==========
    print("\n" + "─" * 70)
    print("BƯỚC 8: NHẬP HÀNG MỚI (vào P0 - liền kề P1)")
    print("─" * 70)

    result = wc.process_inbound(
        order_id=inbound["order_id"], rack_code="RACK-A-001", operator="user123"
    )
    print(f"\n  📦 Nhập item vào P{result['position']} | Item: {result['item_id']}")

    wc.display_rack("RACK-A-001")

    # ========== BƯỚC 6: Xuất hết hàng ==========
    print("\n" + "─" * 70)
    print("BƯỚC 9-10: XUẤT HẾT HÀNG (FIFO - P1 trước P0)")
    print("─" * 70)

    print(f"\n  📤 Xuất 2 items còn lại:")

    for i in range(2):
        result = wc.process_outbound(
            order_id=outbound["order_id"], rack_code="RACK-A-001", operator="user456"
        )
        print(
            f"     {i+1}. Xuất item từ P{result['position']} | "
            f"Item: {result['item_id']}"
        )

    wc.display_rack("RACK-A-001")

    # ========== BƯỚC 7: Báo cáo cuối ==========
    print("\n" + "─" * 70)
    print("BÁO CÁO CUỐI CÙNG")
    print("─" * 70)

    inbound_final = wc.get_inbound_order(inbound["order_id"])
    outbound_final = wc.get_outbound_order(outbound["order_id"])

    print(f"\n  📦 Đơn nhập {inbound['order_id']}:")
    print(f"     • Tổng: {inbound_final['total_items']} items")
    print(f"     • Đã nhập: {inbound_final['received_items']} items")
    print(f"     • Trạng thái: {inbound_final['status'].upper()}")

    print(f"\n  📤 Đơn xuất {outbound['order_id']}:")
    print(f"     • Yêu cầu: {outbound_final['required_items']} items")
    print(f"     • Đã xuất: {outbound_final['picked_items']} items")
    print(f"     • Trạng thái: {outbound_final['status'].upper()}")

    inventory_final = wc.get_product_inventory("PROD-001")
    print(f"\n  📊 Tồn kho cuối:")
    print(f"     • Tổng số items: {inventory_final['total_items']}")

    print("\n" + "=" * 70)
    print("  ✅ DEMO HOÀN THÀNH")
    print("=" * 70 + "\n")


# ================================================================================
# MAIN
# ================================================================================

if __name__ == "__main__":
    """Chạy demo"""
    try:
        demo_complete_workflow()
    except Exception as e:
        print(f"\n❌ Lỗi: {str(e)}")
        import traceback

        traceback.print_exc()
