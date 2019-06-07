from threading import Condition, Lock
from ad_hoc.video_stream import Player, CameraFinished
from ad_hoc.train_classifier import Trainer, TrainerExited

__all__ = ('Controller', )


class Controller(object):

    def run(self):
        condition = Condition()

        trainer = Trainer(train_path="data2/train2/", test_path="data2/test/", condition=condition)
        trainer.train()
        trainer.run_classification()

        player = Player(condition=condition, url='rtsp://admin@192.168.0.100:554/12')
        player.play()

        try:
            with condition:
                while True:
                    condition.wait()

                    try:
                        image_result = player.get_next_frame()
                    except CameraFinished:
                        return

                    if image_result is not None:
                        image, t = image_result
                        print(t, image.get_pixel_format(), image.get_buffer_size())
                        trainer.request_classification(image)

                    try:
                        classification_result = trainer.get_next_classification_result()
                    except TrainerExited:
                        return

                    if classification_result is not None:
                        original_image, (buffer, prediction) = classification_result
                        print('predicted', prediction)

        finally:
            try:
                player.stop()
            finally:
                trainer.stop()


if __name__ == '__main__':
    controller = Controller()
    controller.run()
    print('exited')
