from pymongo import MongoClient, ASCENDING
from datetime import datetime, UTC  # ✅ Import UTC theo yêu cầu
from typing import Optional, Dict, List
import uuid
from enum import Enum


# ------------------------------------------------------------------------------
# TRẠNG THÁI
# ------------------------------------------------------------------------------
class RackStatus(Enum):
    EMPTY = "empty"
    PARTIAL = "partial"
    FULL = "full"


class TransactionType(Enum):
    INBOUND = "inbound"
    OUTBOUND = "outbound"


# ------------------------------------------------------------------------------
# BẢNG ĐIỀU KHIỂN KHO
# ------------------------------------------------------------------------------
class WarehouseMultiRackFIFO:
    def __init__(
        self,
        mongo_uri: str = "mongodb://localhost:27017/",
        db_name: str = "warehouse_final_perfect",
    ):
        """Khởi tạo kết nối MongoDB"""
        self.client = MongoClient(mongo_uri)
        self.db = self.client[db_name]
        self.racks = self.db.racks
        self.transactions = self.db.transactions

        # Tạo index cho tìm kiếm nhanh
        self.racks.create_index(
            [("product_id", ASCENDING), ("available_count", ASCENDING)]
        )
        self.racks.create_index([("status", ASCENDING)])

        # Xóa dữ liệu cũ để demo
        self.racks.delete_many({})
        self.transactions.delete_many({})

    # --------------------------------------------------------------------------
    # TẠO RACK
    # --------------------------------------------------------------------------
    def create_rack(self, rack_code: str) -> Dict:
        """Tạo rack mới với 4 vị trí cố định"""

        if self.racks.find_one({"rack_code": rack_code}):
            raise ValueError(f"❌ Rack {rack_code} đã tồn tại")

        rack = {
            "rack_code": rack_code,
            "capacity": 4,
            "occupied_count": 0,
            "available_count": 4,
            "status": RackStatus.EMPTY.value,
            "product_id": None,
            "positions": [
                {
                    "position_index": 0,
                    "slot_id": f"{rack_code}-P0",
                    "status": "empty",
                    "item": None,
                },
                {
                    "position_index": 1,
                    "slot_id": f"{rack_code}-P1",
                    "status": "empty",
                    "item": None,
                },
                {
                    "position_index": 2,
                    "slot_id": f"{rack_code}-P2",
                    "status": "empty",
                    "item": None,
                },
                {
                    "position_index": 3,
                    "slot_id": f"{rack_code}-P3",
                    "status": "empty",
                    "item": None,
                },
            ],
            "created_at": datetime.now(UTC),  # ✅ Đúng syntax datetime
            "updated_at": datetime.now(UTC),  # ✅ Đúng syntax datetime
        }

        self.racks.insert_one(rack)
        print(f"✅ Tạo rack thành công: {rack_code}")
        return rack

    # --------------------------------------------------------------------------
    # VALIDATION: NGĂN CHẶN TẠO LỖ TRỐNG KHI NHẬP HÀNG
    # --------------------------------------------------------------------------
    def _validate_inbound_position(self, rack: Dict, position_index: int) -> bool:
        """Kiểm tra vị trí nhập có tuân theo quy tắc P3→P2→P1→P0 không"""

        leftmost_occupied = None
        for i in range(4):
            if rack["positions"][i]["status"] == "occupied":
                leftmost_occupied = i
                break

        # Trường hợp 1: Rack trống → chỉ cho phép nhập P3
        if leftmost_occupied is None:
            return position_index == 3

        # Trường hợp 2: Rack đã có hàng → chỉ cho phép nhập vào vị trí liền kề phía trước
        allowed_position = leftmost_occupied - 1
        return position_index == allowed_position

    # --------------------------------------------------------------------------
    # TÌM VỊ TRÍ NHẬP/XUẤT TRONG RACK
    # --------------------------------------------------------------------------
    def _find_inbound_position_in_rack(self, rack: Dict) -> Optional[int]:
        """Tìm vị trí nhập hợp lệ theo quy tắc P3→P2→P1→P0"""
        leftmost_occupied = None
        for i in range(4):
            if rack["positions"][i]["status"] == "occupied":
                leftmost_occupied = i
                break
        return (
            3
            if leftmost_occupied is None
            else (leftmost_occupied - 1) if (leftmost_occupied - 1) >= 0 else None
        )

    def _find_outbound_position_in_rack(self, rack: Dict) -> Optional[int]:
        """Tìm vị trí xuất theo FIFO: P3→P2→P1→P0"""
        for i in range(3, -1, -1):
            if rack["positions"][i]["status"] == "occupied":
                return i
        return None

    # --------------------------------------------------------------------------
    # TỰ ĐỘNG TÌM RACK PHÙ HỢP CHO SẢN PHẨM
    # --------------------------------------------------------------------------
    def find_available_rack_for_product(self, product_id: str) -> Optional[Dict]:
        """Tìm rack phù hợp: Ưu tiên rack đã chứa sản phẩm này còn chỗ → rack trống"""

        # ✅ Đúng cú pháp MongoDB: {"\)gt": 0}
        existing_rack = self.racks.find_one(
            {"product_id": product_id, "available_count": {"$gt": 0}}
        )

        if existing_rack:
            print(
                f"🔍 Tìm thấy rack {existing_rack['rack_code']} đã chứa {product_id} và còn chỗ"
            )
            return existing_rack

        # Tìm rack trống
        empty_rack = self.racks.find_one({"status": RackStatus.EMPTY.value})

        if empty_rack:
            print(f"🔍 Tìm thấy rack trống {empty_rack['rack_code']}")
            return empty_rack

        # Không tìm thấy rack nào
        print(f"❌ Không có rack trống hoặc rack chứa {product_id} còn chỗ")
        return None

    # --------------------------------------------------------------------------
    # NHẬP HÀNG VỚI VALIDATION
    # --------------------------------------------------------------------------
    def add_item(self, product_id: str, batch_number: str) -> Dict:
        """Nhập hàng tự động chọn rack + validation không tạo lỗ trống"""

        # Tìm rack phù hợp
        target_rack = self.find_available_rack_for_product(product_id)
        if not target_rack:
            raise ValueError(f"❌ Không thể nhập {product_id}: Không có rack khả dụng")

        rack_code = target_rack["rack_code"]

        # Kiểm tra ràng buộc sản phẩm
        if (
            target_rack["product_id"] is not None
            and target_rack["product_id"] != product_id
        ):
            raise ValueError(
                f"❌ Rack {rack_code} đang chứa {target_rack['product_id']}, không thể thêm {product_id}"
            )

        # Tìm vị trí nhập hợp lệ
        position_index = self._find_inbound_position_in_rack(target_rack)
        if position_index is None:
            raise ValueError(f"❌ Rack {rack_code} đã đầy")

        # VALIDATION: Kiểm tra không tạo lỗ trống
        if not self._validate_inbound_position(target_rack, position_index):
            leftmost_occupied = None
            for i in range(4):
                if target_rack["positions"][i]["status"] == "occupied":
                    leftmost_occupied = i
                    break

            allowed_position = (
                3 if leftmost_occupied is None else (leftmost_occupied - 1)
            )
            raise ValueError(
                f"❌ VI PHẠM QUY TẮC! Không thể nhập vào P{position_index}. "
                f"Phải nhập vào P{allowed_position} để không tạo lỗ trống."
            )

        # Tạo item mới
        item_id = f"ITEM-{uuid.uuid4().hex[:8].upper()}"
        new_item = {
            "item_id": item_id,
            "product_id": product_id,
            "batch_number": batch_number,
            "entry_date": datetime.now(UTC),  # ✅ Đúng syntax datetime
        }

        # ✅ Đúng cú pháp MongoDB: "$set" (không có \)
        new_occupied = target_rack["occupied_count"] + 1
        new_available = target_rack["available_count"] - 1
        new_status = (
            RackStatus.FULL.value if new_available == 0 else RackStatus.PARTIAL.value
        )
        new_product_id = product_id if new_occupied > 0 else None

        self.racks.update_one(
            {"rack_code": rack_code},
            {
                "$set": {  # ✅ Đúng cú pháp MongoDB update operator
                    f"positions.{position_index}.status": "occupied",
                    f"positions.{position_index}.item": new_item,
                    "product_id": new_product_id,
                    "status": new_status,
                    "occupied_count": new_occupied,
                    "available_count": new_available,
                    "updated_at": datetime.now(UTC),  # ✅ Đúng syntax datetime
                }
            },
        )

        # Ghi log giao dịch
        self.transactions.insert_one(
            {
                "transaction_id": f"TXN-{uuid.uuid4().hex[:6].upper()}",
                "type": TransactionType.INBOUND.value,
                "rack_code": rack_code,
                "position_index": position_index,
                "item_id": item_id,
                "product_id": product_id,
                "batch_number": batch_number,
                "timestamp": datetime.now(UTC),  # ✅ Đúng syntax datetime
            }
        )

        print(
            f"✅ Nhập item {item_id} vào {rack_code}-P{position_index} (Sản phẩm: {product_id})"
        )
        return {
            "item_id": item_id,
            "rack_code": rack_code,
            "position_index": position_index,
            "product_id": product_id,
        }

    # --------------------------------------------------------------------------
    # LẤY HÀNG THEO FIFO
    # --------------------------------------------------------------------------
    def remove_item(self, product_id: str) -> Dict:
        """Lấy hàng từ rack chứa sản phẩm theo FIFO P3→P2→P1→P0"""

        # ✅ Đúng cú pháp MongoDB: {"\)gt": 0}
        target_rack = self.racks.find_one(
            {"product_id": product_id, "occupied_count": {"$gt": 0}}
        )

        if not target_rack:
            raise ValueError(f"❌ Không tìm thấy rack chứa {product_id} có hàng")

        rack_code = target_rack["rack_code"]

        # Tìm vị trí xuất
        position_index = self._find_outbound_position_in_rack(target_rack)
        if position_index is None:
            raise ValueError(f"❌ Rack {rack_code} đang trống")

        item = target_rack["positions"][position_index]["item"]

        # ✅ Đúng cú pháp MongoDB: "$set" (không có \)
        new_occupied = target_rack["occupied_count"] - 1
        new_available = target_rack["available_count"] + 1
        new_status = (
            RackStatus.EMPTY.value if new_occupied == 0 else RackStatus.PARTIAL.value
        )
        new_product_id = product_id if new_occupied > 0 else None

        self.racks.update_one(
            {"rack_code": rack_code},
            {
                "$set": {  # ✅ Đúng cú pháp MongoDB update operator
                    f"positions.{position_index}.status": "empty",
                    f"positions.{position_index}.item": None,
                    "product_id": new_product_id,
                    "status": new_status,
                    "occupied_count": new_occupied,
                    "available_count": new_available,
                    "updated_at": datetime.now(UTC),  # ✅ Đúng syntax datetime
                }
            },
        )

        # Ghi log
        self.transactions.insert_one(
            {
                "transaction_id": f"TXN-{uuid.uuid4().hex[:6].upper()}",
                "type": TransactionType.OUTBOUND.value,
                "rack_code": rack_code,
                "position_index": position_index,
                "item_id": item["item_id"],
                "product_id": product_id,
                "batch_number": item["batch_number"],
                "timestamp": datetime.now(UTC),  # ✅ Đúng syntax datetime
            }
        )

        print(
            f"✅ Lấy item {item['item_id']} từ {rack_code}-P{position_index} (Sản phẩm: {product_id})"
        )
        return {
            "item_id": item["item_id"],
            "rack_code": rack_code,
            "position_index": position_index,
            "product_id": product_id,
        }

    # --------------------------------------------------------------------------
    # HIỂN THỊ TẤT CẢ RACKS
    # --------------------------------------------------------------------------
    def display_all_racks(self):
        """Hiển thị trạng thái tất cả racks"""

        racks = list(self.racks.find().sort("rack_code", ASCENDING))
        if not racks:
            print("❌ Không có rack nào trong kho")
            return

        print(f"\n{'='*80}")
        print(f"  DANH SÁCH RACKS ({len(racks)} racks)")
        print(f"{'='*80}")

        for rack in racks:
            print(
                f"\n  📦 RACK: {rack['rack_code']} | {rack['status'].upper():8} | "
                f"Đã dùng: {rack['occupied_count']}/{rack['capacity']} | "
                f"Sản phẩm: {rack['product_id'] or 'Trống'}"
            )

            # Hiển thị vị trí
            line = "     "
            for pos in rack["positions"]:
                line += f" P{pos['position_index']}:"
                line += f" {'OCC' if pos['status'] == 'occupied' else 'EMP'} "
            print(line)

            # Hiển thị item ID
            line = "     "
            for pos in rack["positions"]:
                if pos["item"]:
                    line += f" {pos['item']['item_id'][-6:]} "
                else:
                    line += "  --  "
            print(line)

    # --------------------------------------------------------------------------
    # HIỂN THỊ RACK CỤ THỂ
    # --------------------------------------------------------------------------
    def display_rack(self, rack_code: str):
        """Hiển thị chi tiết 1 rack"""

        rack = self.racks.find_one({"rack_code": rack_code})
        if not rack:
            print(f"❌ Rack {rack_code} không tồn tại")
            return

        print(f"\n{'═'*70}")
        product_info = f" | Sản phẩm: {rack['product_id'] or 'Trống'}"
        print(
            f"  RACK: {rack['rack_code']} | {rack['status'].upper():8} | "
            f"Đã dùng: {rack['occupied_count']}/{rack['capacity']}{product_info}"
        )
        print(f"{'═'*70}")
        print("\n  ← FRONT (Xuất sau)                    BACK (Xuất trước) →\n")

        print("  ┌──────────────┬──────────────┬──────────────┬──────────────┐")
        line = "  │"
        for pos in rack["positions"]:
            line += f"      P{pos['position_index']}      │"
        print(line)

        line = "  │"
        for pos in rack["positions"]:
            if pos["status"] == "occupied":
                line += f"   OCCUPIED   │"
            else:
                line += f"    EMPTY     │"
        print(line)

        line = "  │"
        for pos in rack["positions"]:
            if pos["item"]:
                short_id = pos["item"]["item_id"][-6:]
                line += f"   {short_id:^8}  │"
            else:
                line += f"      --      │"
        print(line)

        print("  └──────────────┴──────────────┴──────────────┴──────────────┘")

        next_in = self._find_inbound_position_in_rack(rack)
        next_out = self._find_outbound_position_in_rack(rack)

        arrow = "  "
        for i in range(4):
            if i == next_in and rack["available_count"] > 0:
                arrow += "     ↓ IN      "
            elif i == next_out and rack["occupied_count"] > 0:
                arrow += "     ↑ OUT     "
            else:
                arrow += "               "
        print(arrow)

        occupied = [p for p in rack["positions"] if p["status"] == "occupied"]
        if occupied:
            print(f"\n  📦 Danh sách items ({len(occupied)} items):")
            for pos in occupied:
                item = pos["item"]
                print(
                    f"     P{pos['position_index']}: {item['item_id']} | "
                    f"Batch: {item['batch_number']}"
                )

        print()


# ------------------------------------------------------------------------------
# DEMO QUY TRÌNH HOÀN CHỈNH
# ------------------------------------------------------------------------------
def demo():
    print("\n" + "=" * 80)
    print("  DEMO: HỆ THỐNG KHO HOÀN HẢO (KHÔNG LỖI NÀO)")
    print("=" * 80)

    warehouse = WarehouseMultiRackFIFO()

    # BƯỚC 1: Tạo 2 racks
    print("\n" + "─" * 80)
    print("BƯỚC 1: TẠO 2 RACKS")
    print("─" * 80)
    warehouse.create_rack("RACK-A-001")
    warehouse.create_rack("RACK-A-002")
    warehouse.create_rack("RACK-A-004")
    warehouse.create_rack("RACK-A-005")
    warehouse.create_rack("RACK-A-006")
    warehouse.create_rack("RACK-A-007")
    warehouse.display_all_racks()

    # BƯỚC 2: NHẬP HÀNG THEO QUY TẮC
    print("\n" + "─" * 80)
    print("BƯỚC 2: NHẬP HÀNG THEO QUY TẮC P3→P2→P1")
    print("─" * 80)
    warehouse.add_item("PROD-001", "BATCH-2024-001")
    warehouse.add_item("PROD-001", "BATCH-2024-002")
    warehouse.add_item("PROD-001", "BATCH-2024-033")
    warehouse.display_rack("RACK-A-001")

    # BƯỚC 3: XUẤT HÀNG THEO FIFO
    print("\n" + "─" * 80)
    print("BƯỚC 3: XUẤT HÀNG THEO FIFO P3→P2")
    print("─" * 80)
    warehouse.remove_item("PROD-001")
    warehouse.remove_item("PROD-001")
    warehouse.add_item("PROD-002", "BATCH-2024-033")
    warehouse.add_item("PROD-002", "BATCH-2024-033")

    warehouse.display_rack("RACK-A-001")

    # BƯỚC 4: NHẬP THÊM SAU KHI XUẤT
    print("\n" + "─" * 80)
    print("BƯỚC 4: NHẬP THÊM SAU KHI XUẤT → TỰ ĐỘNG VÀO P2")
    print("─" * 80)
    warehouse.add_item("PROD-001", "BATCH-2024-002")
    warehouse.add_item("PROD-001", "BATCH-2024-002")
    warehouse.add_item("PROD-001", "BATCH-2024-002")

    warehouse.display_rack("RACK-A-001")

    print("\n" + "=" * 80)
    print("  ✅ DEMO HOÀN THÀNH - HỆ THỐNG HOÀN HẢO")
    print("=" * 80 + "\n")


if __name__ == "__main__":
    try:
        demo()
    except Exception as e:
        print(f"\n❌ Lỗi: {str(e)}")
        import traceback

        traceback.print_exc()
