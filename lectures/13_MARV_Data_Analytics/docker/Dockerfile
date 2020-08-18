# Copyright 2016 - 2020  Ternaris.
# SPDX-License-Identifier: Apache-2.0

FROM ternaris/marv-robotics
RUN apt-get update && \
    apt-get install -y \
        git \
        protobuf-compiler \
    && rm -rf /var/lib/apt/lists/*

ADD centernet_hg104_512x512_coco17_tpu-8.tar.gz /opt/marv/
ADD tensorflow-2.3.0-cp38-cp38-manylinux2010_x86_64.whl /opt/marv/
RUN chown -R root:root /opt/marv/centernet_hg104_512x512_coco17_tpu-8

RUN cd /opt/marv && \
    echo "Install TensorFlow" && \
    ./bin/python -m pip install tensorflow-2.3.0-cp38-cp38-manylinux2010_x86_64.whl && \
    echo "Install TensorFlow Object Detection API" && \
    git clone --depth 1 https://github.com/tensorflow/models.git /tmp/models && \
    out=/tmp/models/research && \
    protoc $out/object_detection/protos/*.proto --proto_path=$out --python_out=$out && \
    cp $out/object_detection/packages/tf2/setup.py $out && \
    ./bin/python -m pip install $out && \
    cp $out/object_detection/data/mscoco_label_map.pbtxt . && \
    rm -r /tmp/models
