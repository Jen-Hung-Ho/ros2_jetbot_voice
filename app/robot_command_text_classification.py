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

import matplotlib.pyplot as plt
import os
import re
import shutil
import string
import tensorflow as tf
from tensorflow.keras import layers, losses
import json

class TextClassification:
    def __init__(self, train_data_path, test_data_path):
        # Model constants
        self.batch_size = 32
        self.seed = 42
        self.max_features = 20000
        self.sequence_length = 550
        self.embedding_dim = 100
        self.epochs = 100
        self.vectorize_layer = None
        self.model = None
        self.class_labels = None
        self.train_data_path = train_data_path
        self.test_data_path = test_data_path

    #
    # Load text dataset from directory
    #
    def load_data(self):
        raw_train_ds = tf.keras.utils.text_dataset_from_directory(
            self.train_data_path,
            batch_size=self.batch_size,
            validation_split=0.2,
            subset='training',
            seed=self.seed)

        raw_val_ds = tf.keras.utils.text_dataset_from_directory(
            self.train_data_path,
            batch_size=self.batch_size,
            validation_split=0.2,
            subset='validation',
            seed=self.seed)

        raw_test_ds = tf.keras.utils.text_dataset_from_directory(
            self.test_data_path,
            batch_size=self.batch_size)

        self.class_labels = raw_train_ds.class_names
        print("Class labels:", self.class_labels)

        return raw_train_ds, raw_val_ds, raw_test_ds

    # Having looked at our data above, we see that the raw text contains HTML break
    # tags of the form '<br />'. These tags will not be removed by the default
    # standardizer (which doesn't strip HTML). Because of this, we will need to
    # create a custom standardization function.
    # Register the custom standardization function
    @tf.keras.utils.register_keras_serializable(package='Custom', name='custom_standardization')
    def custom_standardization(self, input_data):
        lowercase = tf.strings.lower(input_data)
        stripped_html = tf.strings.regex_replace(lowercase, '<br />', ' ')
        return tf.strings.regex_replace(stripped_html,
                                        '[%s]' % re.escape(string.punctuation),
                                        '')

    # Now that we have our custom standardization, we can instantiate our text
    # vectorization layer. We are using this layer to normalize, split, and map
    # strings to integers, so we set our 'output_mode' to 'int'.
    # Note that we're using the default split function,
    # ---  and the custom standardization defined above. ---
    # and the built-in standardization function.
    # We also set an explicit maximum sequence length, since the CNNs later in our
    # model won't support ragged sequences.
    def build_vectorize_layer(self, raw_train_ds):
        self.vectorize_layer = layers.TextVectorization(
            # standardize=self.custom_standardization,
            max_tokens=self.max_features,
            output_mode='int',
            output_sequence_length=self.sequence_length)

        # Now that the vectorize_layer has been created, call `adapt` on a text-only
        # dataset to create the vocabulary. You don't have to batch, but for very large
        # datasets this means you're not keeping spare copies of the dataset in memory.

        # Let's make a text-only dataset (no labels):
        train_text = raw_train_ds.map(lambda x, y: x)
        self.vectorize_layer.adapt(train_text)

    def vectorize_text(self, text, label):
        text = tf.expand_dims(text, -1)
        return self.vectorize_layer(text), label

    #
    # Build a simple 1D convolutional neural network (CNN) 
    # starting with an Embedding layer.
    #
    def build_model(self):
        # This initializes a sequential model, which is a linear stack of layers.
        self.model = tf.keras.Sequential([
            # It’s used for text data where each word is represented by an integer.
            # Turns positive integers (indexes) into dense vectors of fixed size.
            layers.Embedding(self.max_features, self.embedding_dim),
            # This layer randomly sets 50% of the input units to 0 at each update during training time, which helps prevent overfitting.
            layers.Dropout(0.5),
            # This is a 1D convolutional layer with 128 filters, a kernel size of 7, “valid” padding (no padding), ReLU activation function, and a stride of 3.
            # It helps in extracting features from the input sequence.
            layers.Conv1D(128, 7, padding="valid", activation="relu", strides=3),  # Change kernel size to 7 and strides to 3
            layers.Conv1D(128, 7, padding="valid", activation="relu", strides=3),  # Change kernel size to 7 and strides to 3
            # This layer performs global max pooling operation for temporal data. 
            # It reduces the dimensionality of the input by taking the maximum value over the time dimension.
            layers.GlobalMaxPooling1D(),
            # This is a fully connected (dense) layer with 128 units and ReLU activation function. 
            # It helps in learning complex representations.
            layers.Dense(128, activation="relu"),
            layers.Dropout(0.5),  # Increase dropout rate to 0.5
            # This is the output layer with a number of units equal to the number of classes. 
            # The softmax activation function is used to output a probability distribution over the classes.
            layers.Dense(len(self.class_labels), activation='softmax')
        ])

        self.model.compile(loss=losses.SparseCategoricalCrossentropy(from_logits=False),
                        optimizer='adam',
                        metrics=['accuracy'])


    #
    # Train the model
    #
    def train_model(self, train_ds, val_ds):
        history = self.model.fit(
            train_ds,
            validation_data=val_ds,
            epochs=self.epochs)
        return history

    #
    # Save the trained  model
    #
    def save_export_model(self, lable_file, model_file_name, raw_test_ds):
        export_model = tf.keras.Sequential([
            self.vectorize_layer,
            self.model
        ])

        # Reinitialize the optimizer
        export_model.compile(
            loss=losses.SparseCategoricalCrossentropy(from_logits=False),
            optimizer="adam",
            metrics=['accuracy']
        )

        results = export_model.evaluate(raw_test_ds)
        print(results)
        loss, accuracy = results[:2]
        print(accuracy)

        export_model.class_labels = self.class_labels
        # Save the model with the appropriate file extension
        # Save the model with the .kears extension
        model_file_name = model_file_name + '.keras'
        export_model.save(model_file_name)

        with open(lable_file, 'w') as f:
            json.dump(self.class_labels, f)


def main():

    # Define datasets folder location
    train_data_path = '../data/datasets/train'
    test_data_path = '../data/datasets/test'
    lable_file = '../data/models/class_labels.json'
    model_file_name = '../data/models/ASR_classify_model'

    text_classification = TextClassification(train_data_path, test_data_path)
    raw_train_ds, raw_val_ds, raw_test_ds = text_classification.load_data()
    text_classification.build_vectorize_layer(raw_train_ds)

    # Veecorize the data
    train_ds = raw_train_ds.map(text_classification.vectorize_text)
    val_ds = raw_val_ds.map(text_classification.vectorize_text)
    test_ds = raw_test_ds.map(text_classification.vectorize_text)

    # Do async prefetching / buffering of the data for best performance on GPU.
    AUTOTUNE = tf.data.AUTOTUNE
    train_ds = train_ds.cache().prefetch(buffer_size=AUTOTUNE)
    val_ds = val_ds.cache().prefetch(buffer_size=AUTOTUNE)
    test_ds = test_ds.cache().prefetch(buffer_size=AUTOTUNE)

    # Build model with simple 1D convolutional neural network (CNN)
    text_classification.build_model()
    text_classification.train_model(train_ds, val_ds)
    # Evulate the test data before save it
    text_classification.save_export_model(lable_file, model_file_name, raw_test_ds)

if __name__ == "__main__":
    main()
