# Copyright 2016 - 2020  Ternaris.
# SPDX-License-Identifier: Apache-2.0

import xml.etree.ElementTree as ET
from pathlib import Path

import marv_api as marv
import marv_nodes
import yaml

from .types_capnp import Metadata, Objects


@marv.node(Objects)
@marv.input('dataset', marv_nodes.dataset)
def objects(dataset):
    dataset = yield marv.pull(dataset)
    tracklet_file = next((x for x in dataset.files if x.path.endswith('tracklet_labels.xml')), None)
    if tracklet_file is None:
        return

    root = ET.parse(tracklet_file.path).getroot()
    cars = root.findall(".//*[objectType='Car']")
    yield marv.push({'cars': len(cars)})


@marv.node(Metadata)
@marv.input('dataset', marv_nodes.dataset)
def metadata(dataset):
    dataset = yield marv.pull(dataset)
    dataset_yaml = next((x for x in dataset.files if x.path.endswith('dataset.yaml')), None)
    if dataset_yaml is None:
        return

    dct = yaml.safe_load(Path(dataset_yaml.path).read_text())
    yield marv.push(dct)
