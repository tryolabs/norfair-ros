#!/usr/bin/env python3
import cv2
import message_filters
import norfair
import numpy as np
import rospy
from cv_bridge import CvBridge
from norfair.drawing import Drawable
from norfair_ros.msg import Detections as DetectionsMsg
from sensor_msgs.msg import Image


class VideoWriter:
    """
    This class writes Norfair's output video into a file.
    """

    def get_video_writer(self, output_path: str, camera_reading: dict) -> cv2.VideoWriter:
        """
        Get video writer.

        Parameters
        ----------
        output_path : str
            Path to the output video.
        camera_reading : dict
            Camera reading parameters.

        Returns
        -------
        cv2.VideoWriter
            Video writer.
        """
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        output_size = (
            camera_reading["width"],
            camera_reading["height"],
        )
        video = cv2.VideoWriter(
            output_path,
            fourcc,
            camera_reading["fps"],
            output_size,
        )

        return video

    def write_video(self, image: Image, detections: DetectionsMsg):
        """
        Write video to file.

        Parameters
        ----------
        image : Image
            Message with the image.
        detections : DetectionsMsg
            Message with the detections.
        """

        # Transform DetectionsMsg to Norfair Drawable
        drawables = []
        for detection in detections.detections:
            drawables.append(
                Drawable(
                    points=np.array([point.point for point in detection.points]),
                    label=detection.label,
                    id=detection.id,
                    scores=np.array(detection.scores),
                )
            )

        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

        norfair.draw_boxes(cv_image, drawables=drawables, draw_ids=True)

        self.video.write(cv_image)

    def main(self):
        """
        If output_path is not empty, it will write the output video to a file, indicated by the output_path parameter.
        """
        subscribers = rospy.get_param("video_writer_subscribers")
        camera_reading = subscribers["camera_reading"]
        norfair_detections = subscribers["detections"]

        self.bridge = CvBridge()
        output_path = rospy.get_param("output_path")

        self.video = self.get_video_writer(output_path, camera_reading)

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
