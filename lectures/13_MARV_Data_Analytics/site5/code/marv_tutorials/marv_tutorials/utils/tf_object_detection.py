# Copyright 2016 - 2020  Ternaris.
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path

import numpy as np
import tensorflow as tf
from object_detection.builders import model_builder
from object_detection.utils import config_util, label_map_util, visualization_utils


def make_detect_function(model_dir, labelmap_path):
    configs = config_util.get_configs_from_pipeline_file(str(Path(model_dir, 'pipeline.config')))
    model = model_builder.build(model_config=configs['model'], is_training=False)

    ckpt = tf.compat.v2.train.Checkpoint(model=model)
    ckpt.restore(str(Path(model_dir, 'checkpoint', 'ckpt-0'))).expect_partial()

    @tf.function
    def func(image):
        """Detect objects in image."""
        image, shapes = model.preprocess(image)
        prediction_dict = model.predict(image, shapes)
        detections = model.postprocess(prediction_dict, shapes)

        return detections, prediction_dict, tf.reshape(shapes, [-1])

    label_map = label_map_util.load_labelmap(str(Path(labelmap_path)))
    categories = label_map_util.convert_label_map_to_categories(
        label_map,
        max_num_classes=label_map_util.get_max_label_map_index(label_map),
        use_display_name=True,
    )
    category_index = label_map_util.create_category_index(categories)
    return func, category_index


def detect(image, func):
    input_tensor = tf.convert_to_tensor(np.expand_dims(image, 0), dtype=tf.float32)
    detections, _, _ = func(input_tensor)
    return detections


def visualize(image, detections, category_index):
    image_with_detections = image.copy()
    visualization_utils.visualize_boxes_and_labels_on_image_array(
        image_with_detections,
        detections['detection_boxes'][0].numpy(),
        (detections['detection_classes'][0].numpy() + 1).astype(int),
        detections['detection_scores'][0].numpy(),
        category_index,
        use_normalized_coordinates=True,
        max_boxes_to_draw=200,
        min_score_thresh=.30,
        agnostic_mode=False)

    return image_with_detections
