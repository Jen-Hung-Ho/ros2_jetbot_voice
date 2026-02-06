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
from tensorflow import keras
from tensorflow.keras import layers, losses
import json

class TextClassification:
    def __init__(self, train_data_path, test_data_path):
        # Model constants
        self.batch_size = 32
        self.seed = 42
        self.max_features = 20000
        self.sequence_length = 50  # Reduced for short texts
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

        vocab_size = self.vectorize_layer.vocabulary_size()  # Use actual vocabulary size

        # Sequential model definition with Conv1D for better text classification
        self.model = keras.Sequential([
            keras.layers.Embedding(input_dim=vocab_size, output_dim=128),
            keras.layers.Dropout(0.2),
            keras.layers.Conv1D(128, 7, padding="valid", activation="relu", strides=3),
            keras.layers.Conv1D(128, 7, padding="valid", activation="relu", strides=3),
            keras.layers.GlobalMaxPooling1D(),
            keras.layers.Dense(128, activation="relu"),
            keras.layers.Dropout(0.2),
            keras.layers.Dense(len(self.class_labels), activation="softmax")  # match your dataset labels
        ])

        # Compile the model
        self.model.compile(
            optimizer="adam",
            loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=False),
            metrics=["accuracy"]
        )

        return self.model


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
    def save_export_model(self, label_file, model_file_name, raw_test_ds=None):


        # Build export model: raw text → vectorize → classifier
        export_model = tf.keras.Sequential([
            self.vectorize_layer,
            self.model
        ])

        # Compile only if you want to evaluate
        export_model.compile(
            loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=False),
            optimizer="adam",
            metrics=['accuracy']
        )

        # Optional evaluation
        if raw_test_ds is not None:
            results = export_model.evaluate(raw_test_ds)
            print("Evaluation results:", results)
            loss = results[0]
            accuracy = results[1]
            print(f"Test loss: {loss:.4f}, Test accuracy: {accuracy:.4f}")


        # Save model (choose format)
        # Save the model with the appropriate file extension
        # Save the model with the .kears extension
        model_file_name = model_file_name + '.keras'
        export_model.save(model_file_name)
        # Or: export_model.save(model_file_name + ".keras") if you want Keras v3 format

        # Save labels separately
        with open(label_file, "w") as f:
            json.dump(self.class_labels, f)

        print(f"Model saved to {model_file_name}")
        print(f"Labels saved to {label_file}")


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
