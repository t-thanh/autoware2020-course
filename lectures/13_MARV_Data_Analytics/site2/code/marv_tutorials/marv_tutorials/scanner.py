# Copyright 2016 - 2020  Ternaris.
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path

from marv_api import DatasetInfo


def scan(dirpath, dirnames, filenames):
    """Scan for directories with dataset.yaml files.

    All files in such a directory are treated as dataset files;
    further subdirectories are ignored.
    """
    if 'dataset.yaml' not in filenames:
        return []

    dirnames[:] = []  # do not recurse further
    return [DatasetInfo(Path(dirpath).name, filenames)]
