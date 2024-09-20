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
import json
import string
import re  # Import the re module


class TextClassifier:

    #
    # Initialize the class with load model and labels
    #
    def __init__(self, model_path, labels_path):
        self.custom_objects = {
            'TextVectorization': TextVectorization,
            'custom_standardization': self.custom_standardization
        }
        model_path = model_path + '.keras'
        self.model = tf.keras.models.load_model(model_path)
        # disable custom_standardization -- failed to desearized the model data
        # self.model = tf.keras.models.load_model(model_path, custom_objects=self.custom_objects)

        with open(labels_path, 'r') as f:
            self.class_labels = json.load(f)

    #
    # Having looked at our data above, we see that the raw text contains HTML break
    # tags of the form '<br />'. These tags will not be removed by the default
    # standardizer (which doesn't strip HTML). Because of this, we will need to
    # create a custom standardization function.
    #
    # Register the custom standardization function
    @tf.keras.utils.register_keras_serializable(package='Custom', name='custom_standardization')
    def custom_standardization(self, input_data):
        lowercase = tf.strings.lower(input_data)
        stripped_html = tf.strings.regex_replace(lowercase, '<br />', ' ')
        return tf.strings.regex_replace(stripped_html, '[%s]' % re.escape(string.punctuation), '')
    
    #
    # This function handles the prediction process for the text classifier model
    # 1D convolutional neural network (CNN) model
    #
    def predict(self, min_score, input_text):
        prediction = self.model.predict(tf.constant([input_text]))
        predicted_class_index = tf.argmax(prediction, axis=1).numpy()[0]
        most_fit_score = prediction[0][predicted_class_index]
        predicted_class_label = self.class_labels[predicted_class_index]

        if most_fit_score < min_score:
            return "other", False, most_fit_score
        else:
            return predicted_class_label, True, most_fit_score