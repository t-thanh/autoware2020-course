# Copyright 2016 - 2020  Ternaris.
# SPDX-License-Identifier: Apache-2.0

"""Image filter node adding bounding boxes."""

import marv_api as marv

from .imgsrc import imgsrc
from .utils.tf_object_detection import detect, make_detect_function, visualize


@marv.node()
@marv.input('stream', default=imgsrc)
def bbox(stream):
    """Create videofile from input sream."""
    func, catmap = make_detect_function('/opt/marv/centernet_hg104_512x512_coco17_tpu-8',
                                        '/opt/marv/mscoco_label_map.pbtxt')
    while True:
        img = yield marv.pull(stream)
        if img is None:
            break

        detections = detect(img, func)
        vizimg = visualize(img, detections, catmap)
        yield marv.push(vizimg)
