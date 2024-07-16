#!/usr/bin/env python3
import rospy
from stickler.state_machine import SticklerForTheRules
import smach
import smach_ros

from geometry_msgs.msg import Pose, Point, Quaternion, Polygon, PolygonStamped
from shapely.geometry import Polygon as ShapelyPolygon

from std_msgs.msg import Header

if __name__ == "__main__":
    rospy.init_node("stickler_robocup")

    living_room_area = rospy.get_param("stickler/living_room")

    living_room_pose_param = rospy.get_param("stickler/living_room_pose")
    living_room_pose = Pose(
        position=Point(**living_room_pose_param["position"]),
        orientation = Quaternion(**living_room_pose_param["orientation"])
    )

    forbidden_room_area = rospy.get_param("stickler/forbidden_room")
    forbidden_room_pose_param = rospy.get_param("stickler/forbidden_room_pose")

    forbidden_room_pose = Pose(
        position=Point(**forbidden_room_pose_param["position"]),
        orientation = Quaternion(**forbidden_room_pose_param["orientation"])
    )


    entrance_room_area = rospy.get_param("stickler/entrance")

    stickler = SticklerForTheRules(
        living_room_pose,
        living_room_area,
        forbidden_room_pose,
        forbidden_room_area
    )
    outcome = stickler.execute()
    rospy.loginfo(f"Stickler for the Rules finished with outcome: {outcome}")
    rospy.spin()
