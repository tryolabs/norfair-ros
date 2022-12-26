![Norfair by Tryolabs logo](https://user-images.githubusercontent.com/67343574/207469518-cb59f59f-6677-4414-acdf-38615dfbd285.png)

# ROS package for implementing multi-object tracking using [Norfair](https://github.com/tryolabs/norfair)

# How to use

Norfair is a Python package that implements a multi-object tracking algorithm. This package is designed to be an interface between Norfair and ROS. The package is built into three nodes, `converter`, `video_writer`, and `norfair_ros`.

The typical flow of the package is reading the detections from a detector node, converting them to the Norfair format, and publishing the tracking results.

Norfair offers a drawing API to visualize the tracking results. This package uses this API to draw the tracking results and save them as a video file. This is particularly useful for the debugging process.

We build a [development repository](https://github.com/tryolabs/norfair-ros-dev) where you can find a functional environment running on Docker. This repository pretends to be an easy way to try and learn how to use the Norfair package before integrating it into your workspace.

# Installation

If you like to use the Norfair package in your own ROS environment you can install it in the following way.

## Building

Inside your catkin workspace clone this repo.

```
cd catkin_ws/src
git clone git@github.com:tryolabs/norfair-ros.git
```

After that, build. Depending on the versions of catkin are using you can run:

```
cd ..
catkin_make
```

Or, run this with the new catkin tools.

```
roscd norfair_ros
catkin build
```

## Dependencies

This package is built on Python and depends on [Norfair](https://github.com/tryolabs/norfair). You can install with `pip install norfair`.

# Nodes

This package is built in three nodes, `converter`, `video_writer`, and `norfair_ros`.

The `converter` node is designed to be an interface to unify different input formats to a unique format to the `norfair_ros` node.

You can define your conversions in the `converter` or use the provided implementations. Now [darknet_ros](https://github.com/leggedrobotics/darknet_ros/tree/master/darknet_ros) is supported by the converter.

The converter normalized output is read by the `norfair_ros` node and tracking them as output.

The `video_writer` node is used to save the Norfair output video.

You can find more information about each node, as subscribed/published topics, and parameters in [this](src/nodes_doc.md) specific documentation.

# Example usage

This package has a [launch file](launch/norfair_node.launch) designed to run the three nodes in a single command. A common practice in ROS is to have a startup package that launches all the nodes in a single command. In this case, you can include this launch file in your startup package.

In case you like to launch this package from the command line you can do it in the following way.

```
roslaunch norfair_ros norfair_ros.launch
```

This command initializes the `converter`, `video_writer`, and `norfair_ros` nodes with the default parameters defined in [config files](config/).

As mentioned before, in case you not working with the `darknet_ros` detector, you need to define your own conversions in the `converter` node. This conversion needs to transform your detector format to the Norfair format defined in the message [Detections.msg](msg/Detections.msg).

After that, you can publish your detection to the Norfair inputs topic called `norfair/input`. The node process the detections and publish the tracking information to the topic `norfair/output`.

If you like to use the video writer node, you must publish your image to the topic `camera/rgb/image_raw` using the `Image` type and modify the path to save the output video in the `config/video_writer.yaml` file.

In the case that you like to change the names of the subscribed/published topics, you can do it in the [config files](config/).

# Debugging

We suggest using the `video_writer` node to debug the tracking process.

To enable this capability you have to edit the `config/video_writer.yaml` file and set the `output_path` argument with the desired output path and change the necessary configuration to your particular case in the same file.

The output video will include the bounding boxes and the tracking information.

To prevent an unsynchronized situation you must set properly the publisher rate of your images to prevent the `video_writer` node is faster than the `norfair_ros` node, losing frames.
