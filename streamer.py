import ffmpeg

(
    ffmpeg
    .input('/dev/video0', f='v4l2')
    .output('pipe:', vcodec='rawvideo', pix_fmt='yuv420p')
).run()

