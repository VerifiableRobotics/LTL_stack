from ffpyplayer.player import MediaPlayer
import time
from threading import Thread, Lock

__all__ = ('Player', 'CameraFinished')


class CameraFinished(Exception):
    pass


class Player(object):

    url = ''

    last_image = None

    thread = None

    condition = None

    _finish_playing = False

    _finished_playing = False

    def __init__(self, url='',
                 condition=None, **kwargs):
        super(Player, self).__init__(**kwargs)
        self.url = url
        self.condition = condition

    def play(self):
        if self.thread is not None:
            raise Exception('Already playing')

        self._finished_playing = self._finish_playing = False
        self.thread = Thread(target=self.thread_func)
        self.thread.start()

    def thread_func(self):
        condition = self.condition

        ff_opts = {'f': 'rtsp'}
        player = MediaPlayer(self.url, ff_opts=ff_opts)
        print('hi')

        try:
            while not self._finish_playing:
                frame, val = player.get_frame()
                if val == 'eof':
                    break
                elif frame is None:
                    time.sleep(0.01)
                else:
                    img, t = frame

                    with condition:
                        self.last_image = img, t
                        condition.notify()

                    time.sleep(val)
        finally:
            self._finished_playing = True

    def stop(self):
        thread = self.thread
        self._finish_playing = True
        if thread is not None:
            thread.join()
        self.thread = None

    def get_next_frame(self):
        if self._finished_playing:
            raise CameraFinished

        with self.condition:
            if self.last_image is None:
                return

            image, t = self.last_image
            self.last_image = None

        return image, t

