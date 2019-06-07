from ffpyplayer.player import MediaPlayer
import time
import numpy as np
import cv2
from ad_hoc.train_classifier2 import Trainer, TrainerExited
from skimage.transform import resize
from ffpyplayer.pic import SWScale, Image
from joblib import dump, load

lib_opts = {}
ff_opts = {'f':'rtsp'}
player = MediaPlayer('rtsp://admin@192.168.0.100:554/12',
                     ff_opts=ff_opts)

#trainer = Trainer(train_path="data/trainer_touch/", test_path="data/tester_touch/")
#trainer.train()
print("loading classifier..")
clf = load('filename.joblib')

print("start video...")
while True:
    frame, val = player.get_frame()
    if val == 'eof':
        break
    elif frame is None:
        time.sleep(0.01)
    else:
        img, t = frame
        fmt = img.get_pixel_format()
        scaler = SWScale(*img.get_size(), fmt, ofmt='gray')
        image = scaler.scale(img)
        #print(val, t, img.get_pixel_format(), img.get_buffer_size())
        buffer = np.frombuffer(image.to_bytearray()[0], dtype=np.uint8)
        w, h = image.get_size()
        buffer = np.reshape(buffer, (h, w))
        #buffer = resize(buffer, (100, 200))
        #bu, pred = trainer.predict_image(img)

        bu = resize(buffer, (100, 200), mode='constant')
        bu = bu.flatten()
        bu = bu[np.newaxis, :]

        pred = clf.predict(bu)[0]

        #pred = 1
        if pred == 0:
            k = "not touching"
        elif pred == 1:
            k = "touching"
        #print(pred)
        cv2.putText(buffer, k, (50, 50), cv2.FONT_ITALIC, 0.8, 255)
        cv2.imshow('Video', buffer)
        cv2.waitKey(1)
        #time.sleep(val)