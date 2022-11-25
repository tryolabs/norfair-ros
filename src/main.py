#!/usr/bin/env python3
import numpy as np
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from norfair import Detection, Tracker
from norfair.distances import create_normalized_mean_euclidean_distance
from norfair_ros.msg import Detection as DetectionMsg
from norfair_ros.msg import Detections as DetectionsMsg

DISTANCE_THRESHOLD_CENTROID: float = 0.08


class NorfairNode:
    def process_detections(self, data):
        detections = []
        for detection in data.bounding_boxes:
            detections.append(
                Detection(
                    points=np.array(
                        [detection.xmin, detection.ymin, detection.xmax, detection.ymax]
                    ),
                    scores=np.array([detection.probability]),
                    label=detection.Class,
                )
            )
        tracked_objects = self.tracker.update(detections)

        # Tracked objects to ROS message
        detection_msg = DetectionsMsg()
        detection_msg.detections = []

        for tracked_object in tracked_objects:
            detection_msg.detections.append(
                DetectionMsg(
                    id=tracked_object.id,
                    xmin=tracked_object.last_detection.points[0][0],
                    ymin=tracked_object.last_detection.points[0][1],
                    xmax=tracked_object.last_detection.points[0][2],
                    ymax=tracked_object.last_detection.points[0][3],
                    probability=tracked_object.last_detection.scores[0],
                    Class=tracked_object.last_detection.label,
                )
            )

        self.pub.publish(detection_msg)

    def main(self):
        # Norfair initialization
        distance_function = create_normalized_mean_euclidean_distance(1080, 720)
        distance_threshold = DISTANCE_THRESHOLD_CENTROID

        self.tracker = Tracker(
            distance_function=distance_function,
            distance_threshold=distance_threshold,
        )

        # ROS subscriber and publisher definition
        self.pub = rospy.Publisher("norfair/detections", DetectionsMsg, queue_size=1)
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.process_detections)
        rospy.init_node("norfair")

        rospy.spin()


if __name__ == "__main__":
    try:
        NorfairNode().main()
    except rospy.ROSInterruptException:
        pass
