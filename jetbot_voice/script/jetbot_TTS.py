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

import rclpy  # Python library for ROS 2
import pyaudio

from gtts import gTTS
from io import BytesIO
from pydub import AudioSegment
from threading import Lock
from ctypes import *
from contextlib import contextmanager

from rclpy.node import Node # Handles the creation of nodes
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import String

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


class JetbotTTS(Node):

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'start' and param.type_ == Parameter.Type.BOOL:
                self.start = param.value
                self.get_logger().info('start= {}'.format(bool(param.value)))
            elif param.name == 'input' and param.type_ == Parameter.Type.STRING:
                self.input = param.value
                self.get_logger().info('input= {}'.format(str(param.value)))
                self.TTS_stream(self.input, slow=True, tld='us')

        return SetParametersResult(successful=True)

    def __init__(self):
        super().__init__('Jetbot_TTS')

        self.start = self.declare_parameter('start', False).get_parameter_value().bool_value
        self.input = self.declare_parameter('input', 'jetbot').get_parameter_value().string_value
        self.TTS_topic = self.declare_parameter('TTS_topic', '/TTS/transcripts').get_parameter_value().string_value
        self.device_id= self.declare_parameter('output_device', 10).get_parameter_value().integer_value

        self.get_logger().info('start : {}'.format(self.start))
        self.get_logger().info('TTS_topic : {}'.format(self.TTS_topic))
        self.get_logger().info('device_id : {}'.format(self.device_id))

        self.lock = Lock()

        # Add parameters callback 
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Create the subscriber. This subscriber will receive TTS message
        self.subscription = self.create_subscription(
            String, 
            self.TTS_topic,
            self.TTS_callback,
            10)


    #
    # TTS subscription publish from Jetbot_ASR_client
    #
    def TTS_callback(self, msg):
        if self.start:
            self.TTS_stream(msg.data, slow=False)


    def TTS_stream(self, ASR_string, slow=False, tld='us'):
        with self.lock:
            self.get_logger().info('TTS: "%s"' % ASR_string)

            # ASR_string = msg.data

            mp3_fp = BytesIO()
            # gTTS() - Google-Text-To-Speech
            tts_string = ASR_string
            tts = gTTS(tts_string, lang='en', slow=slow, tld=tld)
            tts.write_to_fp(mp3_fp)
            mp3_fp.seek(0)

            # Load the MP3 file
            # audio = AudioSegment.from_mp3("dusty.wav")
            audio = AudioSegment.from_file(mp3_fp, format="mp3")
            self.get_logger().info("play audio: {}".format(ASR_string))

            # Use PyAudio to play the WAV file
            with noalsaerr():
	            p = pyaudio.PyAudio()

            # Open the WAV file
            stream = p.open(format=p.get_format_from_width(audio.sample_width),
                channels=audio.channels,
                rate=audio.frame_rate,
                output=True,
                output_device_index=self.device_id)

            # Play the WAV file
            stream.write(audio.raw_data)
            stream.stop_stream()
            stream.close()

            self.get_logger().info("TTS done")


def main(args=None):
    
    rclpy.init(args=args)

    JetbotTTS_node = JetbotTTS()    


    try:
        rclpy.spin(JetbotTTS_node)
    except KeyboardInterrupt:
        print('\ncontrol-c: JetbotTTS_node shutting down')
    finally:
        # Destroy the node explictly - don't depend on garbage collector
        JetbotTTS_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()