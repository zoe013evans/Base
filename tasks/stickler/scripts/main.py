#!/usr/bin/env python3
import rospy
from stickler.state_machine import SticklerForTheRules
import smach
import smach_ros

from geometry_msgs.msg import Pose, Point, Quaternion, Polygon, PolygonStamped
from shapely.geometry import Polygon as ShapelyPolygon

from std_msgs.msg import Header

if __name__ == "__main__":
    rospy.init_note("stickler_robocup")
    stickler = SticklerForTheRules()
    outcome = stickler.execute()
    rospy.loginfo(f"Stickler for the Rules finished with outcome: {outcome}")
    rospy.spin()

    