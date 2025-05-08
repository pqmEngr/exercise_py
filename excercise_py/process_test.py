#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime, timedelta, timezone

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from example_interfaces.srv import AddTwoInts
from functools import partial
from robot_interfaces.msg import MissionCurrent
from robot_interfaces.srv import GetInformation, CommandApi
import dateutil.parser
import requests

# from requests import Response


class ProcessSomeThingMeWant(Node):

    def __init__(self):
        super().__init__("location_control_rmf")
        self.timer_cb = MutuallyExclusiveCallbackGroup()
        self.first_programe_excute = True
        timer_period = 1  # seconds

        # self.cli_delete_protocal = self.create_client(CommandApi, "protocal_delete_req")
        # self.cli_get2system = self.create_client(GetInformation, "get_from_system")
        # self.cli_data_update_status = self.create_client(
        #     CommandApi, "update_data_database"
        # )
        # self.expire_now = (datetime.now() - timedelta(seconds=15)).strftime(
        #     "%Y-%m-%d %H:%M:%S.%f"
        # )
        # self.publisher_ = self.create_publisher(String, "demotest", 10)

        # self.url_update_location = "update_location"
        self.current_step = 1
        self.inittial()
        self.timer = self.create_timer(
            timer_period, self.main_loop, callback_group=self.timer_cb
        )

    def inittial(self):
        self.time_runing = datetime.now() + timedelta(days=1)
        self.url_local = "http://127.0.0.1:2000/"

    def communication_continous_to_fleet(self):
        _dcit_request_elevator = {
            "current_map": "pickup_locations",
            "destination_map": "return_locations",
            "step": self.current_step,
        }
        response_fleet = self.sent_mission_to_robot(
            "elevator_request", _dcit_request_elevator
        )
        if response_fleet["code"]:
            self.current_step = self.current_step + 1

        self.get_logger().info('collision_process: "%s"' % response_fleet)

    def sent_mission_to_robot(self, url_tail, mission):

        _url_request = str(self.url_local + url_tail)
        try:
            res = requests.post(
                _url_request,
                json=mission,
                timeout=4,
            )
            response = res.json()
            return response
        except Exception as e:
            # print(e)
            return {"code": 0}

    def main_loop(self) -> None:

        self.expire_now = (datetime.now() - timedelta(minutes=15)).strftime(
            "%Y-%m-%d %H:%M:%S.%f"
        )
        self.communication_continous_to_fleet()
        # self.get_logger().info('collision_process: "%s"' % "had run")


def main(args=None):
    rclpy.init(args=args)
    location_control_rmf = ProcessSomeThingMeWant()
    executor = MultiThreadedExecutor()
    executor.add_node(location_control_rmf)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
