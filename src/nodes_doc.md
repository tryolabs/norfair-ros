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
