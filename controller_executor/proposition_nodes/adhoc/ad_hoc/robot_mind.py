from collections import deque
from playsound import playsound
from joblib import load
import cv2
from PIL import Image


class Robotmind(object):

    last_five_actions = deque([1, 1, 1, 1, 1])

    touch_time = 0

    spoken_touch = False
    spoken_hug = False

    spoken_left = False
    spoken_right = False

    def __init__(self, clf):

        self.touch = False
        self.hug = False
        self.left = False
        self.right = False
        self.speech_clock = 0

        # Load SVM Trained classifier
        print("loading classifier..")
        self.clf = load('classifiers/'+clf)

    def speak_ouch(self):

        self.analyse()

        # speak ouch
        if self.touch and not self.spoken_touch:
            playsound('ouch3.mp3')
            self.spoken_touch = True
        # speak thank you
        elif self.hug and not self.spoken_hug:
            playsound('thank_you.mp3')
            self.spoken_hug = True
        # else speak nothing
        else:
            return

    def speak_help(self):
        self.analyse()

        if self.left and not self.spoken_left:
            playsound('help.mp3')
            self.spoken_left = True
        if self.right and not self.spoken_right:
            playsound('okay.mp3')
            self.spoken_right = True
        else:
            return

    def speak_and_display(self):
        self.analyse()
        if self.touch and self.touch_time == 5:
            im_a = cv2.imread("angry_2.png")  # Read image
            im_s = cv2.imread("sleepy_2.png")
            cv2.imshow("window", im_a)
            cv2.waitKey(10)
            playsound('aaaa.mp3')
            cv2.waitKey(600)
            cv2.imshow("window", im_s)
            cv2.waitKey(1000)
            self.touch_time = 0
        if self.touch:
            im_w = cv2.imread("wake_up_2.png")  # Read image
            im_s = cv2.imread("sleepy_2.png")
            cv2.imshow("window", im_w)
            cv2.waitKey(10)
            playsound('en.mp3')
            cv2.waitKey(1000)
            cv2.imshow("window",im_s)
            cv2.waitKey(1000)

    def update_state(self, buffer):
        # Predict class based on classifier
        pred = self.clf.predict(buffer)[0]

        # Shift predicted action window
        self.last_five_actions.append(pred)
        self.last_five_actions.popleft()

        self.maybe_reset_speech_clock()

        return self.label_pred(pred),pred

    def maybe_reset_speech_clock(self):
        self.speech_clock = self.speech_clock + 1
        if self.speech_clock > 40:

            # speech memory lost
            self.speech_clock = 0
            self.spoken_hug = False
            self.spoken_touch = False
            self.spoken_left = False
            self.spoken_right = False

    def get_current_state(self):
        return self.last_five_actions[4]

    def label_pred(self, pred):
        k = ""
        # three states prediction
        # if pred == 0:
        #     k = "hugging"
        # elif pred == 1:
        #     k = "nothing"
        # elif pred == 2:
        #     k = "touch"

        # whether touch prediction
        if pred == 0:
            k = "not touching"
        elif pred == 1:
            k = "touching"
            #playsound('responding.mp3')

        # choice prediction
        # if pred == 0:
        #     k = "left"
        # elif pred == 1:
        #     k = "no"
        # elif pred == 2:
        #     k = "right"
        return k

    def analyse(self):
        self.touch = False
        self.hug = False
        self.left = False
        self.right = False

        # if self.last_five_actions[4] == 0 and self.last_five_actions[3] == 0:
        if self.last_five_actions[4] == 1 and self.last_five_actions[3] == 1 and self.last_five_actions[2] == 1 and self.last_five_actions[1] == 1:
            self.touch = True
            self.touch_time = self.touch_time+1
        if self.last_five_actions[4] == 0 and self.last_five_actions[3] == 0:
            self.hug = True
        if self.last_five_actions[4] == 0 and self.last_five_actions[3] == 0 and self.last_five_actions[2] == 0 and self.last_five_actions[1] == 0:
            self.left = True
        if self.last_five_actions[4] == 2 and self.last_five_actions[3] == 2 and self.last_five_actions[2] == 2 and self.last_five_actions[1] == 2:
            self.right = True
