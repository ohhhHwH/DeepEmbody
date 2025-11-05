#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from mcp.server.fastmcp import FastMCP
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
import sys
import time


'''

技能名	描述	输入参数	输出参数
s_move_arm_to_pose	控制机械臂移动到指定位姿	pose: Pose6D	success: bool
s_gripper_open	张开夹爪	无	success: bool
s_gripper_close	闭合夹爪	无	success: bool

s_navigate_to_goal	移动到底盘目标点	goal: Pose2D	success: bool

s_grasp_object	感知目标并完成抓取动作	target_name: str	result: str
s_pick_and_place	从物体当前位置抓取并放置至指定位置	target_name: str, place_pose: Pose6D	result: str
s_patrol_and_avoid	沿预定路径巡逻并避障	path: List[Pose2D]	status: str
skl_map_update  当前视角下，识别所有物体，加入语义地图  空   bool：是否成功
skl_map_query_all 查询语义地图所有物体 空 Dict:{obj_name: str，(x, y, z)}
skl_map_query_obj 查询语义地图指定物体 obj_name: str (x, y, z): (float, float, float)
skl_map_add_obj 手动向语义地图中增加物体 obj_name:str, (x, y, z): (float, float, float) Bool: 是否成功
skl_detect_objs 识别指定相机当前视角下，所有物体的类别和坐标 camera_name: str,  Dict: {obj_name, (x, y, z)}
'''

def get_example_tools():
    example_tools = [
        {
            "fn": s_move_arm_to_pose,
            "name": "s_move_arm_to_pose",
            "description": "move arm to specified pose; Args: pose: Pose6D"
        },
        {
            "fn": s_gripper_open,
            "name": "s_gripper_open",
            "description": "open the gripper; Args: None"
        },
        {
            "fn": s_gripper_close,
            "name": "s_gripper_close",
            "description": "close the gripper; Args: None"
        },
        {
            "fn": s_navigate_to_goal,
            "name": "s_navigate_to_goal",
            "description": "navigate to specified goal; Args: goal: Pose2D"
        },
        {
            "fn": s_grasp_object,
            "name": "s_grasp_object",
            "description": "grasp specified object; Args: target_name: str"
        },
        {
            "fn": s_pick_and_place,
            "name": "s_pick_and_place",
            "description": "pick and place object; Args: target_name: str, place_pose : Pose6D"
        },
        {
            "fn": s_patrol_and_avoid,
            "name": "s_patrol_and_avoid",
            "description": "patrol along specified path and avoid obstacles; Args: path: List[Pose2D]"
        },
        {
            "fn": skl_map_update,
            "name": "skl_map_update",
            "description": "update semantic map with current view; Args: None"
        },
        {
            "fn": skl_map_query_all,
            "name": "skl_map_query_all",
            "description": "query all objects in semantic map; Args: None"
        },
        {
            "fn": skl_map_query_obj,
            "name": "skl_map_query_obj",
            "description": "query specified object in semantic map; Args: obj_name: str"
        },
        {
            "fn": skl_map_add_obj,
            "name": "skl_map_add_obj",
            "description": "add object to semantic map; Args: obj_name: str, coordinates: (float, float, float)"
        },
        {
            "fn": skl_detect_objs,
            "name": "skl_detect_objs",
            "description": "detect objects using specified camera; Args: camera_name: str"
        }
        
    ]
    return example_tools


def s_move_arm_to_pose(pose: str) -> bool:
    print(f"s_move_arm_to_pose : Received pose: {pose}")
    print("s_move_arm_to_pose: Simulating arm movement...")
    time.sleep(2)  # Simulate time taken to move the arm
    print("s_move_arm_to_pose: Arm movement completed.")
    return True

def s_gripper_open() -> bool:
    print("s_gripper_open: Simulating gripper opening...")
    time.sleep(1)  # Simulate time taken to open the gripper
    print("s_gripper_open: Gripper opened.")
    return True
def s_gripper_close() -> bool:
    print("s_gripper_close: Simulating gripper closing...")
    time.sleep(1)  # Simulate time taken to close the gripper
    print("s_gripper_close: Gripper closed.")
    return True
def s_navigate_to_goal(goal: str) -> bool:
    print(f"s_navigate_to_goal : Received goal: {goal}")
    print("s_navigate_to_goal: Simulating navigation...")
    time.sleep(3)  # Simulate time taken to navigate
    print("s_navigate_to_goal: Navigation completed.")
    return True
def s_grasp_object(target_name: str) -> str:
    print(f"s_grasp_object : Received target_name: {target_name}")
    print("s_grasp_object: Simulating object grasping...")
    time.sleep(2)  # Simulate time taken to grasp the object
    result = f"Object {target_name} grasped successfully."
    print(f"s_grasp_object: {result}")
    return result
def s_pick_and_place(target_name: str, place_pose: str) -> str:
    print(f"s_pick_and_place : Received target_name: {target_name}, place_pose: {place_pose}")
    print("s_pick_and_place: Simulating pick and place operation...")
    time.sleep(4)  # Simulate time taken to pick and place the object
    result = f"Object {target_name} picked and placed at {place_pose} successfully."
    print(f"s_pick_and_place: {result}")
    return result
def s_patrol_and_avoid(path: str) -> str:
    print(f"s_patrol_and_avoid : Received path: {path}")
    print("s_patrol_and_avoid: Simulating patrol and obstacle avoidance...")
    time.sleep(5)  # Simulate time taken to patrol
    status = "Patrol completed successfully."
    print(f"s_patrol_and_avoid: {status}")
    return status
def skl_map_update() -> bool:
    print("skl_map_update: Simulating semantic map update...")
    time.sleep(2)  # Simulate time taken to update the map
    print("skl_map_update: Semantic map updated.")
    return True
def skl_map_query_all() -> dict:
    print("skl_map_query_all: Simulating semantic map query for all objects...")
    time.sleep(1)  # Simulate time taken to query the map
    result = {
        "object_1": (1.0, 2.0, 0.5),
        "object_2": (3.0, 1.5, 0.0)
    }
    print(f"skl_map_query_all: Retrieved objects: {result}")
    return result
def skl_map_query_obj(obj_name: str) -> tuple:
    print(f"skl_map_query_obj : Received obj_name: {obj_name}")
    print("skl_map_query_obj: Simulating semantic map query for the specified object...")
    time.sleep(1)  # Simulate time taken to query the map
    result = (1.0, 2.0, 0.5)  # Example coordinates
    print(f"skl_map_query_obj: Retrieved coordinates for {obj_name}: {result}")
    return result
def skl_map_add_obj(obj_name: str, coordinates: str) -> bool:
    print(f"skl_map_add_obj : Received obj_name: {obj_name}, coordinates: {coordinates}")
    print("skl_map_add_obj: Simulating adding object to semantic map...")
    time.sleep(1)  # Simulate time taken to add the object
    print(f"skl_map_add_obj: Object {obj_name} added to semantic map.")
    return True
def skl_detect_objs(camera_name: str) -> dict:
    print(f"skl_detect_objs : Received camera_name: {camera_name}")
    print("skl_detect_objs: Simulating object detection...")
    time.sleep(2)  # Simulate time taken to detect objects
    result = {
        "object_1": (1.0, 2.0, 0.5),
        "object_2": (3.0, 1.5, 0.0)
    }
    print(f"skl_detect_objs: Detected objects: {result}")
    return result