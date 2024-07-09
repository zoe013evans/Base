from typing import List, Tuple
import smach
import smach_ros

from lasr_vision_msgs.srv import Recognise
from geometry_msgs.msg import Pose, Point, PointStamped
from shapely.geometry import Polygon
from std_msgs.msg import Header, Empty
import rospy


from lasr_skills import (
    Say,
    GoToLocation,
)

# from stickler import (
#     # Import specific states
# )


class SticklerForTheRules(smach.StateMachine):
    def __init__(
            self,
    ):
        smach.StateMachine.__init__(self,outcomes=["succeeded","failed"])

        smach.StateMachine.add(
            "SAY_START",
            Say(text="Start of Stickler For the Rules. Don't break the rules!"),
            transitions={
                    "succeeded": "SAY_END",
                    "aborted": "SAY_END",
                    "preempted": "SAY_END",
                },

        )

        smach.StateMachine.add(
            "SAY_END",
            Say(text="I am done!"),
            transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "succeeded",
                },

        )