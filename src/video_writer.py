#!/usr/bin/env python3
import cv2
import message_filters
import norfair
import numpy as np
import rospy
from cv_bridge import CvBridge
from norfair_ros.msg import Detections as DetectionsMsg
from sensor_msgs.msg import Image


class VideoWriter:
    """
    This class writes Norfair's output video into a file.
    """

    def write_video(self, image: Image):
        """
        Write video to file.

        Parameters
        ----------
        image : Image
            Message with the image.
        """

        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

        norfair.draw_boxes(cv_image, self.detections, draw_labels=True)

        self.video.write(cv_image)

    def update_detections(self, detections: DetectionsMsg):
        """
        Update the detections.

        Parameters
        ----------
        detections : DetectionsMsg
            Message with the detections.
        """
        # Transform DetectionsMsg to Norfair detections
        self.detections = []
        for detection in detections.detections:
            self.detections.append(
                norfair.Detection(
                    points=np.array([point.point for point in detection.points]),
                    scores=np.array(detection.scores),
                    label=detection.id,
                )
            )

    def main(self):
        """
        If input_video is not empty, it will write the output video to a file, indicated by the input_video parameter.
        """
        subscribers = rospy.get_param("video_writer_subscribers")
        camera_reading = subscribers["camera_reading"]
        norfair_detections = subscribers["detections"]
        self.bridge = CvBridge()

        output_path = rospy.get_param("output_path")
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        output_size = (
            camera_reading["width"],
            camera_reading["height"],
        )  # OpenCV format is (width, height)
        self.video = cv2.VideoWriter(
            output_path,
            fourcc,
            camera_reading["fps"],
            output_size,
        )

        if output_path:
            rospy.init_node("video_writer")

            self.detections = []

            rospy.Subscriber(camera_reading["topic"], Image, self.write_video)
            rospy.Subscriber(norfair_detections["topic"], DetectionsMsg, self.update_detections)

            rospy.spin()


if __name__ == "__main__":
    try:
        VideoWriter().main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Video writer node terminated.")
