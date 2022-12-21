#!/usr/bin/env python
PKG = "norfair_ros"

import unittest

import numpy as np
import rospy
import rostest
from norfair_ros.msg import Detection as DetectionMsg
from norfair_ros.msg import Detections as DetectionsMsg
from norfair_ros.msg import Point


class Test(unittest.TestCase):
    def validate_output(self, bbox: DetectionsMsg):
        self.assertEquals(1, bbox.detections[0].id, "1!=1")

    def test_simple_tracker(self):
        rospy.init_node("test")

        publisher = rospy.Publisher("norfair/input", DetectionsMsg, queue_size=1)
        self.norfair_outputs = rospy.Subscriber(
            "norfair/output", DetectionsMsg, self.validate_output
        )

        # Dummy detections
        seq_bbox = [
            np.array([[0, 0], [10, 10]]),
            np.array([[0, 5], [10, 15]]),
            np.array([[0, 10], [10, 20]]),
            np.array([[0, 15], [10, 25]]),
            np.array([[0, 20], [10, 30]]),
            np.array([[0, 25], [10, 35]]),
        ]
        for bbox in seq_bbox:
            detections_msg = DetectionsMsg()
            detections_msg.detections = [
                DetectionMsg(
                    id=0,
                    label="test",
                    scores=[1, 1],
                    points=[
                        Point([bbox[0][0], bbox[0][1]]),
                        Point([bbox[1][0], bbox[1][0]]),
                    ],
                )
            ]
            publisher.publish(detections_msg)

            # sleep 5hz
            rospy.Rate(5).sleep()


if __name__ == "__main__":
    rostest.rosrun(PKG, "test_bare_bones", Test)
