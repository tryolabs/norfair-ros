#!/usr/bin/env python3
import norfair
import numpy as np
import rospy
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBoxes
from norfair import Detection, Tracker, Video
from norfair_ros.msg import Detection as DetectionMsg
from norfair_ros.msg import Detections as DetectionsMsg
from norfair_ros.msg import Point
from sensor_msgs.msg import Image


class NorfairNode:
    def process_detections(self, bbox):
        self.detections = []
        for detection in bbox.bounding_boxes:
            self.detections.append(
                Detection(
                    points=np.array(
                        [[detection.xmin, detection.ymin], [detection.xmax, detection.ymax]]
                    ),
                    scores=np.array([detection.probability, detection.probability]),
                    label=detection.Class,
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

    def write_video(self, image):
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

        norfair.draw_boxes(cv_image, self.detections)
        norfair.draw_tracked_boxes(cv_image, self.tracked_objects)

        self.video.write(cv_image)

    def main(self):
        # Load parameters
        norfair_setup = rospy.get_param("norfair_setup")
        publisher_topics = rospy.get_param("publisher_topics")
        subscriber_topics = rospy.get_param("subscriber_topics")

        distance_function = norfair_setup["distance_function"]
        distance_threshold = norfair_setup["distance_threshold"]
        input_video = norfair_setup["input_video"]
        norfair_publisher = publisher_topics["norfair_detections"]
        detector_topic = subscriber_topics["detector"]
        image_topic = subscriber_topics["image"]

        # Norfair initialization
        self.tracker = Tracker(
            distance_function=distance_function,
            distance_threshold=distance_threshold,
        )
        self.detections = []
        self.tracked_objects = []
        self.video = Video(input_path=input_video)

        self.bridge = CvBridge()

        rospy.init_node("norfair_node")

        # ROS subscriber and publisher definition
        self.pub = rospy.Publisher(norfair_publisher, DetectionsMsg, queue_size=1)
        rospy.Subscriber(detector_topic, BoundingBoxes, self.process_detections)
        if input_video:
            rospy.Subscriber(image_topic, Image, self.write_video)

        rospy.spin()


if __name__ == "__main__":
    try:
        NorfairNode().main()
    except rospy.ROSInterruptException:
        pass
