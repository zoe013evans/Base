import smach

from lasr_skills import Detect
from lasr_skills.vision import GetImage
from shapely.geometry.polygon import Polygon

from lasr_skills import Detect3DInArea


class CheckRuleForbiddenRoom(smach.StateMachine):

    class CheckForPerson(smach.State):
        def __init__(self):
            smach.State.__init__(
                self, outcomes=["found", "not_found"], input_keys=["detections_3d"]
            )

        def execute(self, userdata):
            if len(userdata.detections_3d):
                return "found"
            else:
                return "not_found"
    def __init__(
        self,
        area_polygon:Polygon,
        image_topic: str = "/xtion/rgb/image_raw",
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["detections"],
        )

        with self: 

            for i in range(0,4):
                smach.StateMachine.add("LOOK_FOR_PERSON_"+str(i),
                                    Detect3DInArea(area_polygon,filter=['person']),
                                    transitions={"succeeded":"CHECK_FOR_PERSON","failed":"failed"})
                smach.StateMachine.add("CHECK_PERSON_"+str(i),
                                       self.CheckForPerson(),
                                       transitions={"found":"FOUND","not_found":"NOT FOUND"})



            


