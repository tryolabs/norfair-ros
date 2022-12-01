#!/usr/bin/env python3
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from norfair_ros.msg import Detection as DetectionMsg
from norfair_ros.msg import Detections as DetectionsMsg
from norfair_ros.msg import Point


class Converter:
    def boundingboxes_to_norfair(self, bboxes: BoundingBoxes):
        detections = []
        for bbox in bboxes.bounding_boxes:
            detections.append(
                DetectionMsg(
                    id=0,
                    label=bbox.Class,
                    scores=[bbox.probability, bbox.probability],
                    points=[
                        Point([bbox.xmin, bbox.ymin]),
                        Point([bbox.xmax, bbox.ymax]),
                    ],
                )
            )

        detections_msg = DetectionsMsg()
        detections_msg.detections = detections

        self.converter_publisher.publish(detections_msg)

    def main(self):
        rospy.init_node("converter")

        # Load parameters
        subscriber_topics = rospy.get_param("subscriber_topics")
        darknet_detector = subscriber_topics["darknet"]

        # ROS subscriber definition
        rospy.Subscriber(darknet_detector, BoundingBoxes, self.boundingboxes_to_norfair)
        self.converter_publisher = rospy.Publisher("norfair/converter", DetectionsMsg, queue_size=1)

        rospy.spin()


if __name__ == "__main__":
    try:
        Converter().main()
    except rospy.ROSInterruptException:
        pass
