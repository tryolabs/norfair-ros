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

    def write_video(self, image: Image):
        """
        Write video to file.

        Parameters
        ----------
        image : Image
            Message with the image.
        """
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

        norfair.draw_boxes(cv_image, self.detections)
        norfair.draw_tracked_boxes(cv_image, self.tracked_objects)

        self.video.write(cv_image)

    def main(self):
        """
        Norfair initialization and subscriber and publisher definition.
        """
        # Load parameters
        distance_function = rospy.get_param("distance_function")
        distance_threshold = rospy.get_param("distance_threshold")
        input_video = rospy.get_param("input_video")
        image_topic = rospy.get_param("image")

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
        self.pub = rospy.Publisher("norfair/detections", DetectionsMsg, queue_size=1)
        rospy.Subscriber("norfair/converter", DetectionsMsg, self.pipeline)
        if input_video:
            rospy.Subscriber(image_topic, Image, self.write_video)

        rospy.spin()


if __name__ == "__main__":
    try:
        NorfairNode().main()
    except rospy.ROSInterruptException:
        pass
