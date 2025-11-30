"""
This script reads the json file that contains the data from a lidar sensor.
The json file is located in the folder 'data' with the name 'real_data.json'.
The structure of the json file is the following:

{
    "data": [
    {
        "scan": [
            <here are 360 float values representing distances in meters>
            <the index of each value represents the angle in degrees from 0 to 359>
            <this angle is relative to the orientation of the robot>
            <the value 0.0 represents that the lidar sensor did not detect any obstacle in that direction>
        ],
        "t": <float timestamp in some dimension. Not important for data processing.>,
        "pose": [
            <the position and orientation of the robot when the scan was taken>
            <x, y, theta are all floats, theta is in radians>
        ]
    },
    ...
    ]
}

The output:
A list of dictionaries, each dictionary contains a RobotPosition key and a list of LidarData.
The structure:

Output = [
    {
        "robot_position": RobotPosition(X=json.data[i].pose[0], Y=json.data[i].pose[1], Theta=json.data[i].pose[2]),
        "scans": [
            LidarData(alpha=json.data[i].indexof(scan[j]), length=json.data[i].scan[j]),
            LidarData(alpha=json.data[i].indexof(scan[j+1]), length=json.data[i].scan[j+1])
            ...
        ]
    },
    ...
]

"""

import json
from pathlib import Path
import math

from .classes.robot_position import RobotPosition
from .classes.lidar_sensor_data import LidarData

HERE = Path(__file__).resolve().parent
DATA_PATH = HERE / "data" / "real_data.json"


def read_json(
    filePath=DATA_PATH, printPath=False
) -> list[dict[RobotPosition, list[LidarData]]]:
    """
    This function reads the specified json file and returns the data in a structured format. \n
    ---
    The output: \n
    A list of dictionaries, where each dictionary contains a RobotPosition and a list of LidarData.
    """
    if printPath:
        print(f"Reading the json file from: {filePath} \n")

    with open(filePath, "r", encoding="utf-8") as jsonFile:
        rawJsonData = json.load(jsonFile)

    outputDataList = []

    for item in rawJsonData["data"]:
        # read the position and scan keys from an item of the data list
        # one item is a dictionary that has the keys: "scan", "t", "pose"
        # the "pose" contains a list of three floats: [X, Y, Theta]
        # the "scan" contains a list of 360 float values representing distances in meters
        position = item["pose"]
        scan = item["scan"]

        robotPosition = RobotPosition(
            X=position[0], Y=position[1], Theta=math.degrees(position[2])
        )

        scans = []
        for i in range(len(scan)):
            # the scan that doesn't produce length is left behind, so that there are no unnecessary data in the output
            if scan[i] == 0.0:
                lidarData = LidarData(
                    alpha=i, length=-1.0
                )  # length -1.0 means no detection
            else:
                lidarData = LidarData(alpha=i, length=scan[i])
            scans.append(lidarData)

        outputDataList.append({"robot_position": robotPosition, "scans": scans})

    return outputDataList
