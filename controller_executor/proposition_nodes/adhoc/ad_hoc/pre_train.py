from train_classifier2 import Trainer, TrainerExited
from joblib import dump, load

## This is to pre-train the classifier and save it as "classifier.joblib"

trainer = Trainer(train_path="data/trainer_touch2/", test_path="data/tester_touch2/")
clf = trainer.train()
dump(clf, 'classifiers/touch.joblib')
