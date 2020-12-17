import ffmpeg

(
    ffmpeg
    .input('/dev/video0', f='v4l2')
    .output('', vcodec='rawvideo')
).run()

