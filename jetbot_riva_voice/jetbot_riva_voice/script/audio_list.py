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

import pyaudio

from ctypes import *
from contextlib import contextmanager

# Define our error handler type
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)

def py_error_handler(filename, line, function, err, fmt):
    pass

c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

# Load the ALSA library
asound = cdll.LoadLibrary('libasound.so')

# Set our error handler
asound.snd_lib_error_set_handler(c_error_handler)

# Suppress the ALSA lib error message 
@contextmanager
def noalsaerr():
    asound = cdll.LoadLibrary('libasound.so')
    asound.snd_lib_error_set_handler(c_error_handler)
    yield
    asound.snd_lib_error_set_handler(None)

def main(args=None):
    # Use PyAudio to play the WAV file
    with noalsaerr():
        p = pyaudio.PyAudio()


    print("----------------------------------------------------")
    print("Audio Input Devices:")
    print("----------------------------------------------------")
    for i in range(p.get_device_count()):
        device_info = p.get_device_info_by_index(i)
        if device_info["maxInputChannels"] > 0:
            print('Input Device {:2d} - \'{}\' (inputs={}) (sample_rate={})'.format(i, device_info['name'], device_info['maxInputChannels'], round(device_info['defaultSampleRate'])))


    print("\n----------------------------------------------------")
    print("Audio Output Devices:")
    print("----------------------------------------------------")
    for i in range(p.get_device_count()):
        device_info = p.get_device_info_by_index(i)
        if device_info["maxOutputChannels"] > 0:
            print('Output Device {:2d} - \'{}\' (outpus={}) (sample_rate={})'.format(i, device_info['name'], device_info['maxOutputChannels'], round(device_info['defaultSampleRate'])))

    p.terminate()

if __name__ == '__main__':
    main()