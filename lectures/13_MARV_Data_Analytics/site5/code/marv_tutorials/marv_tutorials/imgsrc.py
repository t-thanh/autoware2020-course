# Copyright 2016 - 2020  Ternaris.
# SPDX-License-Identifier: Apache-2.0

"""Image stream conversion nodes."""

import marv_api as marv
from marv_robotics.bag import make_deserialize, messages
from marv_ros.img_tools import ImageConversionError, ImageFormatError
from marv_ros.img_tools import imgmsg_to_cv2


@marv.node()
@marv.input('stream', default=marv.select(messages, '/kitti/camera_color_left/image_raw'))
def rosmsg_imgstream(stream):
    """Convert stream of raw ros messages into stream of deserialized messages."""
    deserialize = make_deserialize(stream)
    while True:
        msg = yield marv.pull(stream)
        if msg is None:
            break

        rosmsg = deserialize(msg.data)
        yield marv.push(rosmsg)


@marv.node()
@marv.input('stream', default=rosmsg_imgstream)
def imgsrc(stream):
    """Convert ROS sensor_msgs/Image stream into cv2 image stream."""
    while True:
        rosmsg = yield marv.pull(stream)
        if rosmsg is None:
            break

        try:
            img = imgmsg_to_cv2(rosmsg, 'bgr8')
        except (ImageFormatError, ImageConversionError) as err:
            log = yield marv.get_logger()
            log.error('could not convert image from topic %s: %s ', stream.topic, err)
            raise marv.Abort()

        yield marv.push(img)
