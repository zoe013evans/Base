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
            living_room_pose: Pose,
            living_room_area: Polygon,
            forbidden_room_pose: Pose,
            forbidden_room_area: Polygon,


    ):
        smach.StateMachine.__init__(self,outcomes=["succeeded","failed"])
        self.living_room_pose = living_room_pose
        self.living_room_area = living_room_area
        self.forbidden_room_pose = forbidden_room_pose
        self.forbidden_room_area = forbidden_room_area


        # Loop: 

        with self:

            smach.StateMachine.add(
                "SAY_START",
                Say(text="Start of Stickler For the Rules. Don't break the rules!"),
                transitions={
                        "succeeded": "GO_TO_FORBIDDEN_ROOM",
                        "aborted": "SAY_END",
                        "preempted": "SAY_END",
                    },

            )

            smach.StateMachine.add(
                "GO_TO_FORBIDDEN_ROOM",
                GoToLocation(self.forbidden_room_pose),
                transitions={"succeeded": "ANNOUNCE_FORBIDDEN_ROOM",
                            "failed": "failed"}
        
            )
            smach.StateMachine.add(
                "ANNOUNCE_FORBIDDEN_ROOM",
                Say(text="I'm checking the forbidden room"),
                transitions={"succeeded": "GO_TO_LIVING_ROOM",
                            "aborted": "failed",
                            "preempted": "failed",}
            )
            smach.StateMachine.add(
                "GO_TO_LIVING_ROOM",
                GoToLocation(self.living_room_pose),
                transitions={"succeeded": "ANNOUNCE_LIVING_ROOM",
                            "failed": "failed"}
                
            )

            smach.StateMachine.add(
                "ANNOUNCE_LIVING_ROOM",
                Say(text="I'm checking the living room"),
                transitions={"succeeded": "GO_TO_FORBIDDEN_ROOM",
                            "aborted": "failed",
                            "preempted": "failed"}

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