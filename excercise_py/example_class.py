import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import random


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(String, "missions_hierarchy_fleet", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        msg = String()
        mission_name_1s = str(random.randint(1, 100))
        mission_name_2s = str(random.randint(1, 100))
        mission_name_3s = str(random.randint(1, 100))
        mission_name_4s = str(random.randint(1, 100))

        demo_json = [
            {
                "mission_code": mission_name_1s,
                "line_request": "APM3",
                "part_number": "HV192300-70410V",
                "excute_code": 7,
                "create_time": "2025-05-26T09:01:40.293",
                "mission_status": 18,
                "position_calc": {
                    "location_code": "Empty_5",
                    "robot_point": "LM131",
                    "detect_point": "LM132",
                    "lastAT": {"$date": "2025-05-26T10:58:55.182Z"},
                    "location_status": 3,
                    "location_type": 3,
                    "part_number": "HV192300-70410V",
                    "restore": True,
                    "status_list": {
                        "location_code": "Empty_5",
                        "location_status": 7,
                        "username": "fleet",
                    },
                },
                "actions": [
                    {
                        "name": "action_navigation",
                        "params": {
                            "position": "LM131",
                            "target_lift": None,
                            "pick_point": None,
                            "detect_point": None,
                        },
                    },
                    {
                        "name": "action_lifting",
                        "params": {
                            "position": None,
                            "target_lift": 1,
                            "pick_point": "LM131",
                            "detect_point": "LM132",
                        },
                    },
                    {
                        "name": "action_navigation",
                        "params": {
                            "position": "LM9148",
                            "target_lift": None,
                            "pick_point": None,
                            "detect_point": None,
                        },
                    },
                    {
                        "name": "action_lifting",
                        "params": {
                            "position": None,
                            "target_lift": 2,
                            "pick_point": "LM9148",
                            "detect_point": "LM9146",
                        },
                    },
                    {
                        "name": "action_navigation",
                        "params": {
                            "position": "LM9147",
                            "target_lift": None,
                            "pick_point": None,
                            "detect_point": None,
                        },
                    },
                    {
                        "name": "action_lifting",
                        "params": {
                            "position": None,
                            "target_lift": 1,
                            "pick_point": "LM9147",
                            "detect_point": "LM9145",
                        },
                    },
                    {
                        "name": "action_navigation",
                        "params": {
                            "position": "LM15",
                            "target_lift": None,
                            "pick_point": None,
                            "detect_point": None,
                        },
                    },
                    {
                        "name": "action_lifting",
                        "params": {
                            "position": None,
                            "target_lift": 2,
                            "pick_point": "LM15",
                            "detect_point": "LM15",
                        },
                    },
                ],
            },
            {
                "mission_code": mission_name_2s,
                "line_request": "GMCOIL2",
                "part_number": "HV079640-0673FM",
                "excute_code": 7,
                "create_time": "2025-05-26T09:01:40.293",
                "mission_status": 18,
                "position_calc": None,
                "actions": None,
            },
            {
                "mission_code": mission_name_3s,
                "line_request": "IPV_HONDA1",
                "part_number": "HV136200-74113T",
                "excute_code": 7,
                "create_time": "2025-05-26T12:04:01.023",
                "mission_status": 18,
                "position_calc": None,
                "actions": None,
            },
            {
                "mission_code": mission_name_4s,
                "line_request": "IPV_FORD3",
                "part_number": "HV136200-75901G",
                "excute_code": 7,
                "create_time": "2025-05-27T01:58:21.057",
                "mission_status": 18,
                "position_calc": None,
                "actions": None,
            },
        ]
        # demo_json = [
        mision_name_list = [
            mission_name_1s,
            mission_name_2s,
            mission_name_3s,
            mission_name_4s,
        ]
        msg.data = str(demo_json)
        self.publisher_.publish(msg)
        self.get_logger().info('mision_name_list: "%s"' % mision_name_list)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
