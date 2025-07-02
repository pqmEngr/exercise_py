from flask import Flask, request, jsonify
from flask_cors import CORS

app = Flask(__name__)
CORS(app=app)

import os
import json


package_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "../.."))
json_path_ofset = os.path.join(
    package_dir, "single_handle_system/config", "ofset_machine.json"
)

robot_json = {
    "id": "AGV-001",
    "ipAddress": "107.114.28.24",
    "port": "9888",
    "current_station": None,
    "last_command": "MoveTo",
    "target_id": "DJ3-B06",
    "robot_mode": True,
    "last_time_command": "2025-04-17 16:08:31.449625",
    "amr_out_door": False,
    "excutive_structure": {
        "amr_outdoor": False,
        "hardware_response": {
            "gringer": False,
            "index_in": 5,
            "index_out": 9,
            "position_amr": [12312312],
        },
        "containt_commodity": False,
    },
    "mission_structure": {
        "mission_code": None,
        "mission_status": 0,
        "step_mission": 0,
        "excute_code": None,
        "activity_type": 1,
        "index_mission": 0,
        "action_list": [],
        "mission_list": [],
        "action_error": False,
        "action_name": "",
        "n_task_misison": 0,
        "leisure_time": "2025-04-12 13:48:15.598366",
    },
    "seer_response": {
        "angle": 1.5771,
        "area_ids": [],
        "battery_level": 0.99,
        "blocked": False,
        "charging": False,
        "confidence": 0.7511,
        "create_on": "2025-04-12T13:49:16.360+0700",
        "current_map": "CNC_27_3",
        "current_station": "LM1006",
        "emergency": False,
        "errors": [],
        "last_station": "LM22",
        "reloc_status": 1,
        "ret_code": 0,
        "target_dist": 0.0,
        "target_id": "LM1006",
        "task_status": 4,
        "unfinished_path": [],
        "voltage": -0.0,
        "vx": -0.0,
        "vy": -0.0,
        "x": 61.7614,
        "y": 47.2808,
    },
    "navigation_req": False,
}


@app.route("/status", methods=["GET"])
def get_status_amr():
    try:
        return jsonify(robot_json), 200
    except Exception as e:
        return jsonify({"result": False, "desc": str(e)}), 500


@app.route("/cancel_mission", methods=["POST"])
def cancel_mission():
    content = request.json
    if content:
        print("conten", content.get("id"))
    return {"code": 1}, 200


@app.route("/load_ofset", methods=["GET"])
def load_ofset():
    # content = request.json
    # if content:

    try:
        with open(json_path_ofset, "r") as f:
            dict_point_ofset_robot = json.load(f)
            return dict_point_ofset_robot, 200

    except:
        return {"code": 0}, 200
    return {"code": 0}, 200


@app.route("/alterative_ofset", methods=["POST"])
def alterative_ofset():
    content = request.json
    if content:
        try:
            with open(json_path_ofset, "w") as f:
                json.dump(content, f, indent=4)
                return {"code": 1}, 200
        except:
            return {"code": 0}, 200
    return {"code": 0}, 200


@app.route("/change_quantity_input", methods=["POST"])
def change_quantity_input():
    content = request.json
    if content:
        index_change = content.get("index_in")
        if index_change is not None:
            return {"code": 1, "alternative": index_change}, 200
    return {"code": 0}, 200


@app.route("/change_quantity_output", methods=["POST"])
def change_quantity_output():
    content = request.json
    if content:
        index_change = content.get("index_out")
        if index_change is not None:
            return {"code": 1, "alternative": index_change}, 200
    return {"code": 0}, 200


@app.route("/change_width_gripper", methods=["POST"])
def change_width_gripper():
    content = request.json
    if content:
        index_change = content.get("position")
        if index_change is not None:
            return {"code": 1, "alternative": index_change}, 200

    return {"code": 0}, 200


@app.route("/change_robot_mode", methods=["POST"])
def change_robot_mode():
    content = request.json
    if content:
        index_change = content.get("robot_mode")
        if index_change is not None:

            return {"code": 1, "alternative": index_change}, 200

    return {"code": 0}, 200


@app.route("/change_jig", methods=["POST"])
def change_jig():
    content = request.json
    if content:
        index_change = content.get("jig_mode")
        if index_change is not None:
            return {"code": 1, "alternative": index_change}, 200

    return {"code": 0}, 200


@app.route("/navigation", methods=["POST"])
def navigation():
    content = request.json
    if content:
        index_change = content.get("robot_point")
        if index_change is not None:
            # Robot.navigation({"id": content["position"]})
            return {"code": 1, "alternative": index_change}, 200
    return content, 200


if __name__ == "__main__":
    app.run(debug=True)
