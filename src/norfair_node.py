#!/usr/bin/env python3
import numpy as np
import rospy
from norfair import Detection, Tracker
from norfair_ros.msg import Detection as DetectionMsg
from norfair_ros.msg import Detections as DetectionsMsg
from norfair_ros.msg import Point


class NorfairNode:
    def pipeline(self, bbox: DetectionsMsg):
        """
        Generate Norfair detections and pass them to the tracker.

        Parameters
        ----------
        bbox : DetectionsMsg
            DetectionsMsg message from converter.
        """
        self.detections = []
        for detection in bbox.detections:
            self.detections.append(
                Detection(
                    points=np.array([point.point for point in detection.points]),
                    scores=np.array(detection.scores),
                    label=detection.label,
                )
            )
        self.tracked_objects = self.tracker.update(self.detections)

        # Tracked objects to ROS message
        detection_msg = DetectionsMsg()
        detection_msg.detections = []

        for tracked_object in self.tracked_objects:
            detection_msg.detections.append(
                DetectionMsg(
                    id=tracked_object.id,
                    label=tracked_object.last_detection.label,
                    scores=[score for score in tracked_object.last_detection.scores],
                    points=[Point(point=point) for point in tracked_object.last_detection.points],
                )
            )

        self.pub.publish(detection_msg)

    def main(self):
        """
        Norfair initialization and subscriber and publisher definition.
        """
        # Load parameters
        distance_function = rospy.get_param("distance_function")
        distance_threshold = rospy.get_param("distance_threshold")

        # Norfair initialization
        self.tracker = Tracker(
            distance_function=distance_function,
            distance_threshold=distance_threshold,
        )
        self.detections = []
        self.tracked_objects = []

        rospy.init_node("norfair_node")

        # ROS subscriber and publisher definition
        self.pub = rospy.Publisher("norfair/detections", DetectionsMsg, queue_size=1)
        rospy.Subscriber("norfair/converter", DetectionsMsg, self.pipeline)

        rospy.spin()


if __name__ == "__main__":
    try:
        NorfairNode().main()
    except rospy.ROSInterruptException:
        pass
