# from ffpyplayer.player import MediaPlayer
# import time
# import numpy as np
# import cv2
# from skimage.transform import resize
# from ffpyplayer.pic import SWScale, Image
#
# lib_opts = {}
# ff_opts = {'f':'rtsp'}
# player = MediaPlayer('rtsp://admin@192.168.0.100:554/12',
#                      ff_opts=ff_opts)
#
# while cv2.waitKey(10) < 0:
#     frame, val = player.get_frame()
#     if val == 'eof':
#         break
#     elif frame is None:
#         time.sleep(0.01)
#     else:
#         img, t = frame
#         fmt = img.get_pixel_format()
#         scaler = SWScale(*img.get_size(), fmt, ofmt='gray')
#         image = scaler.scale(img)
#         print(val, t, img.get_pixel_format(), img.get_buffer_size())
#         buffer = np.frombuffer(image.to_bytearray()[0], dtype=np.uint8)
#         w, h = image.get_size()
#         buffer = np.reshape(buffer, (h, w))
#         #buffer = resize(buffer, (100, 200))
#         cv2.imshow('Video', buffer)
#         time.sleep(val)