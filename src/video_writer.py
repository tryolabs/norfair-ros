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

    def write_video(self, image: Image, detections: DetectionsMsg):
        """
        Write video to file.

        Parameters
        ----------
        image : Image
            Message with the image.
        """
        # Transform DetectionsMsg to Norfair detections
        norfair_detections = []
        for detection in detections.detections:
            norfair_detections.append(
                norfair.Detection(
                    points=np.array([point.point for point in detection.points]),
                    scores=np.array(detection.scores),
                    label=detection.label,
                )
            )

        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

        norfair.draw_boxes(cv_image, norfair_detections)

        self.video.write(cv_image)

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

            image_sub = message_filters.Subscriber(camera_reading["topic"], Image)
            detections_sub = message_filters.Subscriber(norfair_detections["topic"], DetectionsMsg)

            ts = message_filters.TimeSynchronizer([image_sub, detections_sub], 2)
            ts.registerCallback(self.write_video)

            rospy.spin()


if __name__ == "__main__":
    try:
        VideoWriter().main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Video writer node terminated.")
