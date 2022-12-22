![Norfair by Tryolabs logo](https://user-images.githubusercontent.com/67343574/207469518-cb59f59f-6677-4414-acdf-38615dfbd285.png)

# ROS package for implementing multi-object tracking using [Norfair](https://github.com/tryolabs/norfair)

# How to use

We build a [development repository](https://github.com/tryolabs/norfair-ros-dev) where you can find a functional environment running on Docker. This repository pretends to be an easy way to try and learn how to use the Norfair package before integrating it into your workspace.

# Installation

If you like to use the Norfair package in your own ROS environment you can install it in the following way.

## Building

Inside your catkin workspace clone this repo.

```
cd catkin_ws/src
git clone git@github.com:tryolabs/norfair-ros.git
```

After that, build.

```
cd ..
catkin_make
```

## Dependencies

This package needs ROS, if you do not have ROS installed [here](http://wiki.ros.org/ROS/Installation) you can find a guide to install it.

Also, [Norfair](https://github.com/tryolabs/norfair) is required. You can install with `pip install norfair`.

# Nodes

This package is built in three nodes, `converter`, `video_writer`, and `norfair_ros`.

The `converter` node is designed to be an interface to unify different input formats to a unique format to the `norfair_ros` node.

You can define your conversions in the `converter` or use the provided implementations. Now [darknet_ros](https://github.com/leggedrobotics/darknet_ros/tree/master/darknet_ros) is supported by the converter.

The converter normalized output is read by the `norfair_ros` node and tracking them as output.

The `video_writer` node is uses to save the Norfair output video.

## `converter`

## Topics

### Subscribed

- Detector: `darknet_ros/bounding_boxes`

If you like to add different detectors, you must subscribe to their output here and define a function to convert the format used by the detector to the one required by Norfair, defined in the `Detection.msg` file.

### Published

- Converter output: `norfair/input`

## Parameters

The parameters are defined in the `config/converter.yaml` file.

## `norfair_ros`

## Topics

### Subscribed

This node is subscribed only to the converter output.

- Converter output: `norfair/input`

### Published

After adding the tracking capability to the detections, Norfair published this as output.

- Norfair output: `norfair/output`

## Parameters

The parameters are defined in the `config/norfair.yaml` file.

## `video_writer`

## Topics

### Subscribed

This node needs the image and the tracking output. For this reason, it is subscribed to the image topic and the Norfair output topic. In the case of the [development repository](https://github.com/tryolabs/norfair-ros-dev) it is subscribe to the following topics.

- Image: `camera/rgb/image_raw`

- Norfair output: `norfair/output`

This node uses `TimeSynchronizer` to synchronize the data from the image and the Norfair output. You must set the correct frequency to the node that publishes the image to prevent losing frames.

Once this node receives the data, it saves the frame with the drawn bounding boxes and the tracking information into a video file.

## Parameters

The parameters are defined in the `config/video_writer.yaml` file.

# Debugging

We suggest using the `video_writer` node to debug the tracking process.

To enable this capability you have to edit the `config/video_writer.yaml` file and set the `output_path` argument with the desired output path and change the necessary configuration to your particular case in the same file.
