#!/usr/bin/env python3
import norfair
import numpy as np
import rospy
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBoxes
from norfair import Detection, Tracker, Video
from norfair.distances import create_normalized_mean_euclidean_distance
from norfair_ros.msg import Detection as DetectionMsg
from norfair_ros.msg import Detections as DetectionsMsg
from sensor_msgs.msg import Image

DISTANCE_THRESHOLD_CENTROID: float = 0.08


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

        rospy.loginfo(self.detections)
        rospy.loginfo(self.tracked_objects)

        # Tracked objects to ROS message
        detection_msg = DetectionsMsg()
        detection_msg.detections = []

        for tracked_object in self.tracked_objects:
            detection_msg.detections.append(
                DetectionMsg(
                    id=tracked_object.id,
                    xmin=tracked_object.last_detection.points[0][0],
                    ymin=tracked_object.last_detection.points[0][1],
                    xmax=tracked_object.last_detection.points[1][0],
                    ymax=tracked_object.last_detection.points[1][1],
                    probability=tracked_object.last_detection.scores[0],
                    Class=tracked_object.last_detection.label,
                )
            )

        self.pub.publish(detection_msg)

    def write_video(self, image):
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

        rospy.loginfo(self.detections)
        rospy.loginfo(self.tracked_objects)

        norfair.draw_boxes(cv_image, self.detections)
        norfair.draw_tracked_boxes(cv_image, self.tracked_objects)

        self.video.write(cv_image)

    def main(self):
        # Norfair initialization
        distance_function = create_normalized_mean_euclidean_distance(1080, 720)
        distance_threshold = DISTANCE_THRESHOLD_CENTROID

        self.tracker = Tracker(
            distance_function=distance_function,
            distance_threshold=distance_threshold,
        )

        self.detections = []
        self.tracked_objects = []

        self.bridge = CvBridge()
        self.video = Video(input_path="/root/catkin_ws/src/publisher/src/example.mp4")

        rospy.init_node("norfair_node")

        # ROS subscriber and publisher definition
        self.pub = rospy.Publisher("norfair/detections", DetectionsMsg, queue_size=1)
        rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, self.process_detections)
        rospy.Subscriber("camera/rgb/image_raw", Image, self.write_video)

        rospy.spin()


if __name__ == "__main__":
    try:
        NorfairNode().main()
    except rospy.ROSInterruptException:
        pass
