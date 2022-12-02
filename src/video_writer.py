#!/usr/bin/env python3
import message_filters
import norfair
import numpy as np
import rospy
from cv_bridge import CvBridge
from norfair import Video
from norfair_ros.msg import Detections as DetectionsMsg
from sensor_msgs.msg import Image


class VideoWriter:
    """
    This class write the Norfair's output video into a file.
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
        image_topic = rospy.get_param("image")
        input_video = rospy.get_param("input_video")
        self.video = Video(input_path=input_video)
        self.bridge = CvBridge()

        if input_video:
            rospy.init_node("video_writer")

            image_sub = message_filters.Subscriber(image_topic, Image)
            detections_sub = message_filters.Subscriber("norfair/detections", DetectionsMsg)

            ts = message_filters.ApproximateTimeSynchronizer([image_sub, detections_sub], 10, 10)
            ts.registerCallback(self.write_video)

            rospy.spin()


if __name__ == "__main__":
    try:
        VideoWriter().main()
    except rospy.ROSInterruptException:
        pass
