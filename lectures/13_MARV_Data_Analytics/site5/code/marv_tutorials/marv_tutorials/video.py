# Copyright 2016 - 2020  Ternaris.
# SPDX-License-Identifier: Apache-2.0

"""Video sink."""

from subprocess import PIPE

import marv_api as marv
from marv.types import File, Section
from marv.utils import popen

# from .imgsrc import imgsrc as default_stream
from .bbox import bbox as default_stream


@marv.node(File)
@marv.input('stream', default=default_stream)
def video_sink(stream):
    """Create videofile from input sream."""
    framerate = 10
    encoder = None
    while True:
        img = yield marv.pull(stream)
        if img is None:
            break

        if not encoder:
            video = yield marv.make_file('camera.webm')
            height, width = img.shape[0:2]
            ffargs = [
                'ffmpeg',
                '-f', 'rawvideo',
                '-pixel_format', 'bgr24',
                '-video_size', f'{width}x{height}',
                '-framerate', f'{framerate}',
                '-i', '-',
                '-c:v', 'libvpx-vp9',
                '-pix_fmt', 'yuv420p',
                '-loglevel', 'error',
                '-threads', '8',
                '-speed', '4',
                '-y',
                video.path,
            ]
            encoder = popen(ffargs, stdin=PIPE)

        encoder.stdin.write(img)

    if encoder:
        encoder.stdin.close()
        encoder.wait()
        yield marv.push(video)


@marv.node(Section)
@marv.input('title', default='Video')
@marv.input('src', default=video_sink)
def video_section(src, title):
    """Section displaying a video player."""
    videofile = yield marv.pull(src)
    if videofile is None:
        return

    yield marv.push({
        'title': title,
        'widgets': [{
            'title': 'Camera',
            'video': {'src': videofile.relpath},
        }],
    })
