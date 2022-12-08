![Norfair by Tryolabs logo](https://raw.githubusercontent.com/tryolabs/norfair/master/docs/img/logo.svg)

[![Hugging Face Spaces](https://img.shields.io/badge/%F0%9F%A4%97%20Hugging%20Face-Spaces-blue)](https://huggingface.co/spaces/tryolabs/norfair-demo)
[![Open in Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/tryolabs/norfair/blob/master/demos/colab/colab_demo.ipynb)

![PyPI - Python Versions](https://img.shields.io/pypi/pyversions/norfair)
[![PyPI](https://img.shields.io/pypi/v/norfair?color=green)](https://pypi.org/project/norfair/)
[![Documentation](https://img.shields.io/badge/api-reference-blue?logo=readthedocs)](https://tryolabs.github.io/norfair/)
[![Board](https://img.shields.io/badge/project-board-blue?logo=github)](https://github.com/tryolabs/norfair/projects/1)
![Build status](https://github.com/tryolabs/norfair/workflows/CI/badge.svg?branch=master)
[![DOI](https://zenodo.org/badge/276473370.svg)](https://zenodo.org/badge/latestdoi/276473370)
[![License](https://img.shields.io/github/license/tryolabs/norfair)](https://github.com/tryolabs/norfair/blob/master/LICENSE)

# A customizable lightweight Python library for real-time multi-object tracking

## [Here](https://github.com/tryolabs/norfair) is the Norfair repository.

Norfair is now available also in the ROS ecosystem. With Norfair you can add tracking capability to different ROS detectors with a little effort.

# How to use

We build a [development repo](https://github.com/tryolabs/norfair-ros-dev) where you can find a functional environment running on Docker. This repository pretends to be an easy way to try and learn to use the Norfair node before integrating it into your own workspace.

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

Also, [Norfair](https://github.com/tryolabs/norfair) is required.

# Nodes

This package is built in two nodes, `converter` and `norfair_ros`.

The `converter` node is designed to be an interface to unify different input formats to a unique format to the `norfair_ros` node.

You can define your conversions in the `converter` or use the provided implementations. Now [darknet_ros](https://github.com/leggedrobotics/darknet_ros/tree/master/darknet_ros) is supported by the converter.

The converter normalized output is read by the `norfair_ros` node and tracking them as output.

## `converter`

## Topics

### Subscribed

- Detector: `darknet_ros/bounding_boxes`

If you like to add different detectors, you need to subscribe to his output here and define a function to convert the format used by the detector to the one required by Norfair, defined in the `Detection.msg` file.

### Published

- Converter output: `norfair/converter`

## Parameters

The parameters are defined in the `config/converter.yaml` file.

## `norfair_ros`

## Topics

### Subscribed

This node is subscribed only to the converter output.

- Converter output: `norfair/converter`

### Published

After adding the tracking capability to the detections, Norfair published this as output.

- Norfair output: `norfair/detections`

## Parameters

The parameters are defined in the `config/norfair.yaml` file.

# Debugging

The `norfair-ros` also includes a node called `video_writer`. With this node, you can save a video with the Norfair outputs and watch the tracking results easier.

This node has to subscribe to the same topic that publishes the images to the detector. In the development repository, we call `publisher`. This node needs the detections also, and for this reason, is subscribed to the Norfair output topic.

Combining these data we can write the information in the video and generate a new file with the output.

To enable this capability you have to edit the `config/video_writer.yaml` file and set the `output_path` argument with the desired output path and change the necessary configuration to your particular case in the same file.
