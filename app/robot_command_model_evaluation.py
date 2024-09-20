#!/usr/bin/env python3
#
# Copyright (c) 2024, Jen-Hung Ho 
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#
#
# Reference:
# This code is inspired by the example from Keras: 
# https://keras.io/examples/nlp/text_classification_from_scratch/
#

import tensorflow as tf
from tensorflow.keras.layers import TextVectorization
from termcolor import colored
import json
import string
import re  # Import the re module
import unittest

from jetbot_riva_voice.include.text_classifier_utility import TextClassifier

# unit test
class TestTextClassifier(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        model_path = '../data/models/ASR_classify_model'
        labels_path = '../data/models/class_labels.json'
        cls.classifier = TextClassifier(model_path, labels_path)
        cls.examples = [
            "turn to right.",
            "turn to left.",
            "What do you see in camera?",
            "follow me.",
            "go forward.",
            "move backward.",
            "stop all actions",
            "start the action",
            "Start self driving mode",
            "Describe what do you see in image?",
            "How are you today?",
            "Hello What is your name?",
            "How many states in the United States?"
        ]
        cls.expected_results = [
            "cmd_right",
            "cmd_left",
            "cmd_vision",
            "cmd_follow",
            "cmd_forward",
            "cmd_backward",
            "cmd_stop",
            "cmd_start",
            "cmd_self-driving",
            "cmd_vision",
            "other",
            "other",
            "other"
        ]

    def test_predictions(self):
        pass_count = 0
        for i, example in enumerate(self.examples):
            label, result, score = self.classifier.predict(0.7, example)
            expected_label = self.expected_results[i]
            if label == expected_label:
                pass_count += 1
                # print(f"PASS: Input: '{example}' => Predicted Label: '{label}', Expected Label: '{expected_label}', Result: '{score}'")
                print(colored(f"PASS: Input: '{example}' => Predicted Label: '{label}', Expected Label: '{expected_label}', Result: '{score}'", 'green'))
                if score < 0.7 and label == 'other':
                    # print(f"PASS: chat topic: '{example}' => Predicted Label: '{label}', Expected Label: '{expected_label}', Result: '{score}'")
                    print(colored(f"PASS: chat topic: '{example}' => Predicted Label: '{label}', Expected Label: '{expected_label}', Result: '{score}'", 'magenta'))

            else:
                # print(f"FAIL: Input: '{example}' => Predicted Label: '{label}', Expected Label: '{expected_label}', Result: '{score}'")
                print(colored(f"FAIL: Input: '{example}' => Predicted Label: '{label}', Expected Label: '{expected_label}', Result: '{score}'", 'red'))

        print("===================================================")
        total_tests = len(self.examples)
        print(f"Test Summary: Passed {pass_count}/{total_tests} tests")
        print("===================================================")


def simple_test():

    # Define data mode file loacation
    model_path = '../data/models/ASR_classify_model'
    labels_path = '../data/models/class_labels.json'
    classifier = TextClassifier(model_path, labels_path)


    # Sample test cases
    examples = [
        "turn to right.",
        "turn to left.",
        "What do you see in camera?",
        "follow me.",
        "go forward.",
        "move backward.",
        "stop all actions",
        "start the action",
        "Start self driving mode",
        "Describe what do you see in image?",
        "How many states in the United States?"
    ]

    print("============================================================")
    print(classifier.class_labels)
    print("============================================================")

    # Run predictions for each example
    for example in examples:
    
        label, result, score = classifier.predict(0.7, example)
        if result:
            print(f"Input: '{example}' => Predicted Label: '{label}', Result: '{score}'")
        else:
            print("Chat topic: -----------------------")
            print(f"Input: '{example}' => Predicted Label: '{label}', Result: '{score}'")


if __name__ == "__main__":
    unittest.main()