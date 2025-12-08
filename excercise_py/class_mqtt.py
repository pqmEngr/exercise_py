"""
Demo RackManager - FIFO Warehouse System
"""

from pymongo import MongoClient
from datetime import datetime, timedelta
from typing import Optional, Dict, Any
from bson import ObjectId


class MongoDBConnection:
    def __init__(
        self,
        connection_string: str = "mongodb://localhost:27017/",
        database_name: str = "warehouse_demo",
    ):
        self.client = MongoClient(connection_string)
        self.db = self.client[database_name]

    def get_collection(self, collection_name: str):
        return self.db[collection_name]


class RackManager:
    """Quản lý rack và FIFO logic"""

    def __init__(self, db_connection: MongoDBConnection):
        self.db = db_connection
        self.racks = self.db.get_collection("racks")
        self.transactions = self.db.get_collection("transactions")

    def _calculate_next_inbound_position(self, rack: Dict) -> Optional[int]:
        """
        Tính vị trí nhập hàng tiếp theo
        Logic: Tìm position EMPTY từ BACK → FRONT (index 3 → 2 → 1 → 0)
        """
        for i in range(rack["capacity"] - 1, -1, -1):
            if rack["positions"][i]["status"] == "empty":
                return i
        return None  # Rack đầy

    def _calculate_next_outbound_position(self, rack: Dict) -> Optional[int]:
        """
        Tính vị trí xuất hàng tiếp theo (FIFO)
        Logic: Tìm position có sequence_number NHỎ NHẤT (hàng cũ nhất)
        """
        occupied_positions = [
            (i, pos)
            for i, pos in enumerate(rack["positions"])
            if pos["status"] == "occupied" and pos["sequence_number"] is not None
        ]

        if not occupied_positions:
            return None  # Rack trống

        # Sắp xếp theo sequence_number (nhỏ nhất = cũ nhất = xuất trước)
        occupied_positions.sort(key=lambda x: x[1]["sequence_number"])
        return occupied_positions[0][0]

    def _get_rack_status(self, rack: Dict) -> str:
        """Tính status của rack dựa trên positions"""
        occupied_count = sum(
            1 for pos in rack["positions"] if pos["status"] == "occupied"
        )

        if occupied_count == 0:
            return "empty"
        elif occupied_count >= rack["capacity"]:
            return "full"
        else:
            return "available"

    def _get_oldest_batch(self, rack: Dict) -> Optional[str]:
        """Tìm batch cũ nhất trong rack"""
        occupied = [
            pos
            for pos in rack["positions"]
            if pos["status"] == "occupied" and pos["sequence_number"] is not None
        ]

        if not occupied:
            return None

        oldest = min(occupied, key=lambda x: x["sequence_number"])
        return oldest["batch_number"]

    def inbound_to_rack(
        self,
        warehouse_id: str,
        rack_id: str,
        product_id: str,
        batch_number: str,
        quantity: int,
        unit: str,
        operator_id: str,
        reference_document: str,
        expiry_date: Optional[datetime] = None,
        notes: str = "",
        manual_position: Optional[int] = None,
    ) -> Dict[str, Any]:
        """
        Nhập hàng vào rack

        Args:
            manual_position: Nếu None → tự động (BACK→FRONT), nếu có giá trị → nhập thủ công vào position đó

        Returns:
            {
                "success": bool,
                "message": str,
                "rack_id": str,
                "position_index": int,
                "transaction_id": str
            }
        """
        # Lấy rack
        rack = self.racks.find_one({"_id": ObjectId(rack_id)})

        if not rack:
            return {
                "success": False,
                "message": f"Không tìm thấy rack {rack_id}",
                "rack_id": None,
                "transaction_id": None,
            }

        # Kiểm tra rack có được gán cho sản phẩm này chưa
        if rack["assigned_product_id"] is None:
            # Lần đầu nhập vào rack trống → gán product
            is_first_inbound = True
        elif rack["assigned_product_id"] != product_id:
            return {
                "success": False,
                "message": f"Rack đã được gán cho sản phẩm {rack['assigned_product_id']}",
                "rack_id": rack_id,
                "transaction_id": None,
            }
        else:
            is_first_inbound = False

        # Xác định vị trí nhập hàng
        if manual_position is not None:
            # Nhập thủ công
            if manual_position < 0 or manual_position >= rack["capacity"]:
                return {
                    "success": False,
                    "message": f"Vị trí {manual_position} không hợp lệ (0-{rack['capacity']-1})",
                    "rack_id": rack_id,
                    "transaction_id": None,
                }

            if rack["positions"][manual_position]["status"] != "empty":
                return {
                    "success": False,
                    "message": f"Vị trí {manual_position} đã có hàng",
                    "rack_id": rack_id,
                    "transaction_id": None,
                }

            inbound_position = manual_position
            is_manual = True
        else:
            # Tự động: tìm từ BACK → FRONT
            inbound_position = self._calculate_next_inbound_position(rack)
            is_manual = False

            if inbound_position is None:
                return {
                    "success": False,
                    "message": "Rack đã đầy",
                    "rack_id": rack_id,
                    "transaction_id": None,
                }

        # Tăng sequence_counter
        sequence_number = rack["sequence_counter"] + 1

        # Tạo transaction
        transaction_data = {
            "transaction_type": "inbound",
            "warehouse_id": warehouse_id,
            "rack_id": rack_id,
            "position_index": inbound_position,
            "product_id": product_id,
            "batch_number": batch_number,
            "quantity": quantity,
            "unit": unit,
            "is_first_inbound": is_first_inbound,
            "is_manual_position": is_manual,
            "sequence_number": sequence_number,
            "operator_id": operator_id,
            "reference_document": reference_document,
            "transaction_date": datetime.utcnow(),
            "notes": notes,
            "created_at": datetime.utcnow(),
        }

        transaction_result = self.transactions.insert_one(transaction_data)
        transaction_id = str(transaction_result.inserted_id)

        # Update position
        update_data = {
            f"positions.{inbound_position}.batch_number": batch_number,
            f"positions.{inbound_position}.quantity": quantity,
            f"positions.{inbound_position}.unit": unit,
            f"positions.{inbound_position}.status": "occupied",
            f"positions.{inbound_position}.inbound_date": datetime.utcnow(),
            f"positions.{inbound_position}.expiry_date": expiry_date,
            f"positions.{inbound_position}.sequence_number": sequence_number,
            "sequence_counter": sequence_number,
            "total_quantity": rack["total_quantity"] + quantity,
            "updated_at": datetime.utcnow(),
        }

        # Nếu là lần đầu, gán product cho rack
        if is_first_inbound:
            update_data["assigned_product_id"] = product_id
            update_data["assigned_date"] = datetime.utcnow()

        # Thực hiện update
        self.racks.update_one({"_id": ObjectId(rack_id)}, {"$set": update_data})

        # Lấy rack sau khi update để tính status
        updated_rack = self.racks.find_one({"_id": ObjectId(rack_id)})
        new_status = self._get_rack_status(updated_rack)

        self.racks.update_one(
            {"_id": ObjectId(rack_id)}, {"$set": {"status": new_status}}
        )

        return {
            "success": True,
            "message": "Nhập hàng thành công",
            "rack_id": rack_id,
            "rack_code": rack["rack_code"],
            "position_index": inbound_position,
            "sequence_number": sequence_number,
            "transaction_id": transaction_id,
            "is_first_inbound": is_first_inbound,
            "is_manual": is_manual,
        }

    def outbound_from_rack(
        self,
        warehouse_id: str,
        rack_id: str,
        product_id: str,
        quantity: int,
        operator_id: str,
        reference_document: str,
        notes: str = "",
        manual_position: Optional[int] = None,
    ) -> Dict[str, Any]:
        """
        Xuất hàng từ rack

        Args:
            manual_position: Nếu None → tự động (FIFO theo sequence_number), nếu có → xuất từ position đó

        Returns:
            {
                "success": bool,
                "message": str,
                "rack_id": str,
                "position_index": int,
                "batch_number": str,
                "quantity_outbound": int,
                "remaining_in_position": int,
                "transaction_id": str
            }
        """
        # Lấy rack
        rack = self.racks.find_one({"_id": ObjectId(rack_id)})

        if not rack:
            return {
                "success": False,
                "message": f"Không tìm thấy rack {rack_id}",
                "rack_id": None,
                "transaction_id": None,
            }

        # Kiểm tra product
        if rack["assigned_product_id"] != product_id:
            return {
                "success": False,
                "message": f"Rack không chứa sản phẩm {product_id}",
                "rack_id": rack_id,
                "transaction_id": None,
            }

        # Xác định vị trí xuất hàng
        if manual_position is not None:
            # Xuất thủ công
            if manual_position < 0 or manual_position >= rack["capacity"]:
                return {
                    "success": False,
                    "message": f"Vị trí {manual_position} không hợp lệ",
                    "rack_id": rack_id,
                    "transaction_id": None,
                }

            if rack["positions"][manual_position]["status"] != "occupied":
                return {
                    "success": False,
                    "message": f"Vị trí {manual_position} không có hàng",
                    "rack_id": rack_id,
                    "transaction_id": None,
                }

            outbound_position = manual_position
            is_manual = True
        else:
            # Tự động: FIFO theo sequence_number
            outbound_position = self._calculate_next_outbound_position(rack)
            is_manual = False

            if outbound_position is None:
                return {
                    "success": False,
                    "message": "Rack không có hàng để xuất",
                    "rack_id": rack_id,
                    "transaction_id": None,
                }

        position = rack["positions"][outbound_position]

        # Kiểm tra số lượng
        if quantity > position["quantity"]:
            return {
                "success": False,
                "message": f"Số lượng yêu cầu ({quantity}) > số lượng có sẵn ({position['quantity']})",
                "rack_id": rack_id,
                "transaction_id": None,
            }

        batch_number = position["batch_number"]
        remaining_quantity = position["quantity"] - quantity

        # Tạo transaction
        transaction_data = {
            "transaction_type": "outbound",
            "warehouse_id": warehouse_id,
            "rack_id": rack_id,
            "position_index": outbound_position,
            "product_id": product_id,
            "batch_number": batch_number,
            "quantity": quantity,
            "unit": position["unit"],
            "is_manual_position": is_manual,
            "sequence_number": position["sequence_number"],
            "operator_id": operator_id,
            "reference_document": reference_document,
            "transaction_date": datetime.utcnow(),
            "notes": notes,
            "created_at": datetime.utcnow(),
        }

        transaction_result = self.transactions.insert_one(transaction_data)
        transaction_id = str(transaction_result.inserted_id)

        # Update position
        update_data = {
            "total_quantity": rack["total_quantity"] - quantity,
            "updated_at": datetime.utcnow(),
        }

        if remaining_quantity > 0:
            # Còn hàng trong position
            update_data[f"positions.{outbound_position}.quantity"] = remaining_quantity
        else:
            # Hết hàng → làm trống position
            update_data[f"positions.{outbound_position}.batch_number"] = None
            update_data[f"positions.{outbound_position}.quantity"] = 0
            update_data[f"positions.{outbound_position}.unit"] = None
            update_data[f"positions.{outbound_position}.status"] = "empty"
            update_data[f"positions.{outbound_position}.inbound_date"] = None
            update_data[f"positions.{outbound_position}.expiry_date"] = None
            update_data[f"positions.{outbound_position}.sequence_number"] = None

        # Thực hiện update
        self.racks.update_one({"_id": ObjectId(rack_id)}, {"$set": update_data})

        # Lấy rack sau update
        updated_rack = self.racks.find_one({"_id": ObjectId(rack_id)})
        new_status = self._get_rack_status(updated_rack)

        # Kiểm tra nếu rack trống hoàn toàn → unassign product
        if new_status == "empty":
            self.racks.update_one(
                {"_id": ObjectId(rack_id)},
                {
                    "$set": {
                        "status": "empty",
                        "assigned_product_id": None,
                        "assigned_date": None,
                        "unassigned_date": datetime.utcnow(),
                        "sequence_counter": 0,
                    }
                },
            )
        else:
            self.racks.update_one(
                {"_id": ObjectId(rack_id)}, {"$set": {"status": new_status}}
            )

        return {
            "success": True,
            "message": "Xuất hàng thành công",
            "rack_id": rack_id,
            "rack_code": rack["rack_code"],
            "position_index": outbound_position,
            "batch_number": batch_number,
            "quantity_outbound": quantity,
            "remaining_in_position": remaining_quantity,
            "transaction_id": transaction_id,
            "is_manual": is_manual,
            "rack_now_empty": new_status == "empty",
        }

    def get_rack_info(self, rack_id: str) -> Optional[Dict]:
        """Lấy thông tin rack với tính toán động"""
        rack = self.racks.find_one({"_id": ObjectId(rack_id)})

        if not rack:
            return None

        # Tính toán động
        next_inbound = self._calculate_next_inbound_position(rack)
        next_outbound = self._calculate_next_outbound_position(rack)
        oldest_batch = self._get_oldest_batch(rack)

        # Thêm thông tin tính toán vào kết quả
        rack["calculated_info"] = {
            "next_inbound_position": next_inbound,
            "next_outbound_position": next_outbound,
            "oldest_batch": oldest_batch,
        }

        return rack

    def print_rack_visual(self, rack_id: str):
        """In visualization của rack"""
        rack = self.get_rack_info(rack_id)

        if not rack:
            print(f"❌ Không tìm thấy rack {rack_id}")
            return

        calc = rack["calculated_info"]

        print("\n" + "=" * 70)
        print(
            f"  RACK: {rack['rack_code']} | {rack['status'].upper():8} | Occupied: {sum(1 for p in rack['positions'] if p['status'] == 'occupied')}/{rack['capacity']}"
        )
        print("=" * 70)
        print()
        print("  ← FRONT (Xuất sau)                    BACK (Xuất trước) →")
        print()

        # Header
        header = "  ┌"
        for i in range(rack["capacity"]):
            header += "──────────────"
            if i < rack["capacity"] - 1:
                header += "┬"
        header += "┐"
        print(header)

        # Position index
        pos_line = "  │"
        for i in range(rack["capacity"]):
            pos_line += f"      P{i}      │"
        print(pos_line)

        # Status
        status_line = "  │"
        for pos in rack["positions"]:
            status = pos["status"].upper()
            status_line += f"   {status:^8}   │"
        print(status_line)

        # Item info
        item_line = "  │"
        for pos in rack["positions"]:
            if pos["status"] == "occupied":
                batch = pos["batch_number"][-8:] if pos["batch_number"] else "--"
                item_line += f"   -{batch}  │"
            else:
                item_line += f"      --      │"
        print(item_line)

        # Footer
        footer = "  └"
        for i in range(rack["capacity"]):
            footer += "──────────────"
            if i < rack["capacity"] - 1:
                footer += "┴"
        footer += "┘"
        print(footer)

        # Arrows
        arrow_line = "  "
        for i in range(rack["capacity"]):
            if calc["next_inbound_position"] == i:
                arrow_line += "     ↓ IN     "
            elif calc["next_outbound_position"] == i:
                arrow_line += "     ↑ OUT    "
            else:
                arrow_line += "              "
        print(arrow_line)

        # Details
        print()
        occupied = [p for p in rack["positions"] if p["status"] == "occupied"]
        if occupied:
            print(f"  📦 Chi tiết ({len(occupied)} items):")
            for pos in sorted(occupied, key=lambda x: x["sequence_number"]):
                print(
                    f"     P{pos['position_index']}: {pos['batch_number']} | Qty: {pos['quantity']} {pos['unit']} | Seq: {pos['sequence_number']}"
                )

        print()


# ==================== DEMO ====================


def create_demo_rack(db: MongoDBConnection) -> str:
    """Tạo rack demo"""
    rack_data = {
        "warehouse_id": "WH001",
        "rack_code": "RACK-A-001",
        "zone": "A",
        "row": 1,
        "column": 1,
        "assigned_product_id": None,
        "assigned_date": None,
        "unassigned_date": None,
        "capacity": 4,
        "status": "empty",
        "sequence_counter": 0,
        "total_quantity": 0,
        "positions": [
            {
                "position_index": i,
                "slot_id": f"RACK-A-001-P{i}",
                "batch_number": None,
                "quantity": 0,
                "unit": None,
                "status": "empty",
                "inbound_date": None,
                "expiry_date": None,
                "sequence_number": None,
            }
            for i in range(4)
        ],
        "created_at": datetime.utcnow(),
        "updated_at": datetime.utcnow(),
    }

    result = db.get_collection("racks").insert_one(rack_data)
    return str(result.inserted_id)


def demo():
    """Demo RackManager"""

    # Kết nối DB
    db = MongoDBConnection()
    rack_mgr = RackManager(db)

    # Xóa dữ liệu cũ
    db.get_collection("racks").delete_many({})
    db.get_collection("transactions").delete_many({})

    print("=" * 70)
    print("  DEMO: WAREHOUSE FIFO SYSTEM")
    print("=" * 70)

    # 1. Tạo rack
    print("\n──────────────────────────────────────────────────────────────────────")
    print("BƯỚC 1: TẠO RACK")
    print("──────────────────────────────────────────────────────────────────────")
    rack_id = create_demo_rack(db)
    print(f"✅ Tạo rack: RACK-A-001")

    rack_mgr.print_rack_visual(rack_id)

    # 2. Nhập hàng lần 1 (tự động - vào P3)
    print("\n──────────────────────────────────────────────────────────────────────")
    print("BƯỚC 2: NHẬP HÀNG LẦN 1 (Tự động - BACK → FRONT)")
    print("──────────────────────────────────────────────────────────────────────")
    result = rack_mgr.inbound_to_rack(
        warehouse_id="WH001",
        rack_id=rack_id,
        product_id="PROD001",
        batch_number="BATCH-2024-001",
        quantity=100,
        unit="piece",
        operator_id="USER001",
        reference_document="IN-001",
        expiry_date=datetime.utcnow() + timedelta(days=365),
    )
    print(f"✅ {result['message']}")
    print(
        f"   Position: P{result['position_index']} | Sequence: {result['sequence_number']}"
    )
    print(f"   First inbound: {result['is_first_inbound']}")

    rack_mgr.print_rack_visual(rack_id)

    # 3. Nhập hàng lần 2 (tự động - vào P2)
    print("\n──────────────────────────────────────────────────────────────────────")
    print("BƯỚC 3: NHẬP HÀNG LẦN 2 (Tự động)")
    print("──────────────────────────────────────────────────────────────────────")
    result = rack_mgr.inbound_to_rack(
        warehouse_id="WH001",
        rack_id=rack_id,
        product_id="PROD001",
        batch_number="BATCH-2024-002",
        quantity=150,
        unit="piece",
        operator_id="USER001",
        reference_document="IN-002",
    )
    print(
        f"✅ {result['message']} | Position: P{result['position_index']} | Seq: {result['sequence_number']}"
    )

    rack_mgr.print_rack_visual(rack_id)

    # 4. Nhập hàng lần 3 (tự động - vào P1)
    print("\n──────────────────────────────────────────────────────────────────────")
    print("BƯỚC 4: NHẬP HÀNG LẦN 3 (Tự động)")
    print("──────────────────────────────────────────────────────────────────────")
    result = rack_mgr.inbound_to_rack(
        warehouse_id="WH001",
        rack_id=rack_id,
        product_id="PROD001",
        batch_number="BATCH-2024-003",
        quantity=120,
        unit="piece",
        operator_id="USER001",
        reference_document="IN-003",
    )
    print(
        f"✅ {result['message']} | Position: P{result['position_index']} | Seq: {result['sequence_number']}"
    )

    rack_mgr.print_rack_visual(rack_id)

    # 5. Xuất hàng lần 1 (FIFO - sẽ lấy từ P3 vì sequence_number = 1)
    print("\n──────────────────────────────────────────────────────────────────────")
    print("BƯỚC 5: XUẤT HÀNG THEO FIFO (sequence_number nhỏ nhất)")
    print("──────────────────────────────────────────────────────────────────────")
    result = rack_mgr.outbound_from_rack(
        warehouse_id="WH001",
        rack_id=rack_id,
        product_id="PROD001",
        quantity=50,
        operator_id="USER002",
        reference_document="OUT-001",
    )
    print(f"✅ {result['message']}")
    print(f"   Position: P{result['position_index']} | Batch: {result['batch_number']}")
    print(
        f"   Xuất: {result['quantity_outbound']} | Còn lại: {result['remaining_in_position']}"
    )

    rack_mgr.print_rack_visual(rack_id)

    # 6. Xuất hết position P3
    print("\n──────────────────────────────────────────────────────────────────────")
    print("BƯỚC 6: XUẤT HẾT P3")
    print("──────────────────────────────────────────────────────────────────────")
    result = rack_mgr.outbound_from_rack(
        warehouse_id="WH001",
        rack_id=rack_id,
        product_id="PROD001",
        quantity=50,
        operator_id="USER002",
        reference_document="OUT-002",
    )
    print(
        f"✅ P{result['position_index']} đã empty, FIFO tự động chuyển sang position tiếp theo"
    )

    rack_mgr.print_rack_visual(rack_id)

    # 7. Nhập thủ công vào P0
    print("\n──────────────────────────────────────────────────────────────────────")
    print("BƯỚC 7: NHẬP THỦ CÔNG VÀO P0")
    print("──────────────────────────────────────────────────────────────────────")
    result = rack_mgr.inbound_to_rack(
        warehouse_id="WH001",
        rack_id=rack_id,
        product_id="PROD001",
        batch_number="BATCH-2024-004",
        quantity=200,
        unit="piece",
        operator_id="USER001",
        reference_document="IN-004",
        manual_position=0,  # Nhập thủ công vào P0
    )
    print(f"✅ {result['message']} | Manual: {result['is_manual']}")
    print(
        f"   Position: P{result['position_index']} | Seq: {result['sequence_number']}"
    )

    rack_mgr.print_rack_visual(rack_id)

    # 8. Xuất hàng (FIFO vẫn ưu tiên sequence_number nhỏ nhất = P2)
    print("\n──────────────────────────────────────────────────────────────────────")
    print("BƯỚC 8: XUẤT HÀNG - FIFO VẪN LẤY THEO SEQUENCE_NUMBER")
    print("──────────────────────────────────────────────────────────────────────")
    result = rack_mgr.outbound_from_rack(
        warehouse_id="WH001",
        rack_id=rack_id,
        product_id="PROD001",
        quantity=150,
        operator_id="USER002",
        reference_document="OUT-003",
    )
    print(
        f"✅ Xuất từ P{result['position_index']} (sequence={rack_mgr.racks.find_one({'_id': ObjectId(rack_id)})['positions'][result['position_index']].get('sequence_number', 'N/A')})"
    )
    print(
        f"   → FIFO đúng vì P{result['position_index']} có sequence_number nhỏ hơn P0"
    )

    rack_mgr.print_rack_visual(rack_id)

    # 9. Xuất thủ công từ P0
    print("\n──────────────────────────────────────────────────────────────────────")
    print("BƯỚC 9: XUẤT THỦ CÔNG TỪ P0")
    print("──────────────────────────────────────────────────────────────────────")
    result = rack_mgr.outbound_from_rack(
        warehouse_id="WH001",
        rack_id=rack_id,
        product_id="PROD001",
        quantity=100,
        operator_id="USER002",
        reference_document="OUT-004",
        manual_position=0,  # Xuất thủ công từ P0
    )
    print(
        f"✅ Xuất thủ công từ P{result['position_index']} | Manual: {result['is_manual']}"
    )

    rack_mgr.print_rack_visual(rack_id)

    # 10. Xuất hết hàng
    print("\n──────────────────────────────────────────────────────────────────────")
    print("BƯỚC 10: XUẤT HẾT HÀNG TRONG RACK")
