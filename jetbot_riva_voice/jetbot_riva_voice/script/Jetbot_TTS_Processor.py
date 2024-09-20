# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import threading
import time
import asyncio

# RIVA client 
import pyaudio
import riva.client
import riva.client.audio_io

from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType, SetParametersResult
from std_msgs.msg import String
from threading import Lock

from jetbot_riva_voice.include.node_parameter_utility import NodeParamTools

class riva_tts_processor(Node):

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'start' and param.type_ == Parameter.Type.BOOL:
                self.start = param.value
                self.get_logger().info('start= {}'.format(bool(param.value)))
            elif param.name == 'command' and param.type_ == Parameter.Type.STRING:
                self.cmd = param.value
                self.get_logger().info('command= {}'.format(str(param.value)))

        return SetParametersResult(successful=True)

    def __init__(self, name):
        super().__init__(name)

        self.TTS_topic = self.declare_parameter('TTS_topic', "/TTS/transcripts").get_parameter_value().string_value 
        self.chat_topic = self.declare_parameter('chat_topic', "/chatbot/response").get_parameter_value().string_value 
        self.ASR_node = self.declare_parameter('ASR_node', '/Riva_ASR_processor').get_parameter_value().string_value
        self.RIVA_URL = self.declare_parameter('url', "localhost:50051").get_parameter_value().string_value
        self.index = self.declare_parameter('index', 26).get_parameter_value().integer_value
        self.streaming_chunk = self.declare_parameter('streaming_chunk', 16000).get_parameter_value().integer_value

        self.get_logger().info("=========================================")
        self.get_logger().info("Jetbot ASR processor :{} start".format(name))
        self.get_logger().info("TTS_topic       : {}".format(self.TTS_topic))
        self.get_logger().info("response_reopic : {}".format(self.chat_topic))
        self.get_logger().info('ASR_node        : {}'.format(self.ASR_node))
        self.get_logger().info("RIVA url        : {}".format(self.RIVA_URL))
        self.get_logger().info("index           : {}".format(self.index))
        self.get_logger().info("streaming chunk : {}".format(self.streaming_chunk))
        self.get_logger().info("=========================================")

        self.lock = Lock()
        self.subscription = self.create_subscription(String, self.TTS_topic, self.TTS_callback, 10)
        self.chat_subscription = self.create_subscription(String, self.chat_topic, self.chat_callback, 10)

        self.msg = String()
        self.i = 0

        # Add parameters callback 
        self.add_on_set_parameters_callback(self.parameter_callback)

        # self.init_ros_nodes()
        self.node_param_util = NodeParamTools(self, executor)

        self.riva_init()

        # self.thread = threading.Thread(target=self.ASR_processor)
        # self.thread.start()

    #
    # Remove nodes for get/set parameter service call
    #
    def cleanup(self):
        # clean up set_param_node, get_param_node
        self.node_param_util.cleanup()
        pass

    def riva_init(self):
        self.p = pyaudio.PyAudio()
        default_device_info = riva.client.audio_io.get_default_input_device_info()
        self.get_logger().debug("Rivai default info:{}".format(default_device_info))
        # default_index = None if default_device_info is None else default_device_info['index']
        if default_device_info is not None and int(default_device_info['maxOutputChannels']) > 0:
            self.audio_index = default_device_info['index']
            self.get_logger().info("use default - ignore user input")
        else:
            self.audio_index = self.index
        default_device = self.p.get_device_info_by_index(self.audio_index)
        self.sample_rate = int(default_device['defaultSampleRate'])

        self.get_logger().info("==============================================")
        self.get_logger().info("Audio default index     : {}".format(self.audio_index))
        self.get_logger().info("Max input ouput channels: [{} - {}]".format(default_device['maxInputChannels'], default_device_info['maxOutputChannels']))
        self.get_logger().info("sample rate             : {}".format(self.sample_rate))
        # riva.client.audio_io.list_input_devices()
        self.get_logger().info("==============================================")

         # Initialize RIVA
        # auth = riva.client.Auth(args.ssl_cert, args.use_ssl, args.server, args.metadata)
        auth = riva.client.Auth(None, False, self.RIVA_URL, None)
        self.tts_service = riva.client.SpeechSynthesisService(auth)

        self.get_logger().info("==============================================")
        self.get_logger().info(" RIVA speech synthesis service")
        self.get_logger().info("==============================================")

    def list_audio_devices(self):
        self.get_logger().info("==============================================")
        self.get_logger().info(" AUDIO DEVICES: ")
        self.get_logger().info("==============================================")
        for i in range(self.p.get_device_count()):
            dev = self.p.get_device_info_by_index(i)
            self.get_logger().info(f"{dev['index']:2d}: {dev['name']:50s} (inputs={dev['maxInputChannels']:<3d} outputs={dev['maxOutputChannels']:<3d} sampleRate={int(dev['defaultSampleRate'])})")
        self.get_logger().info("==============================================")

    def chat_callback(self, msg):

        self.TTS_callback(msg)
        # Turn of mute mode ASR node
        passfail, value = self.node_param_util.try_get_node_parameters(self.ASR_node, 'start')
        if passfail == True:
            self.get_logger().info('Jetbot chat node start:{}'.format(value.bool_value))
            if not value.bool_value:
                # delay 3 seconds for wait ASR finish current sound decoding then turn on the ASR 
                time.sleep(2.0)
                self.node_param_util.try_set_node_parameters(self.ASR_node, 'start', type=ParameterType.PARAMETER_BOOL, value=True)
            else:
                self.get_logger().info('Jetbot chat node start == true')


    def TTS_callback(self, msg):
        with self.lock:
            self.get_logger().info('TTS_callback: [{}]'.format(msg.data))
            msg_str = msg.data
            nchannels = 1
            sampwidth = 2
            sound_stream, out_f = None, None
            start = time.time()
            try:
                sound_stream = riva.client.audio_io.SoundCallBack(
                    self.audio_index, nchannels=nchannels, sampwidth=sampwidth, framerate=self.sample_rate
                )

                self.get_logger().info(" Generating audio for request.. \n msg:{}".format(msg_str))
                responses = self.tts_service.synthesize_online(
                    msg_str, None, "en-US", sample_rate_hz=self.sample_rate,
                    audio_prompt_file=None, quality=20
                )

                first = True
                for resp in responses:
                    stop = time.time()
                    if first:
                        self.get_logger().info(" Time to first audio: {:.3f}s".format(stop - start))
                        first = False
                    if sound_stream is not None:
                        sound_stream(resp.audio)
            except Exception as e:
                self.get_logger().error('An error occured: {}'.format(e))
            finally:
                if sound_stream is not None:
                    sound_stream.close()
                    self.get_logger().info("Close riva service sound stream")

def main(args=None):
    rclpy.init(args=None)

    global executor

    executor = rclpy.executors.MultiThreadedExecutor()

    JetbotTTS_node = riva_tts_processor('Riva_TTS_processor')
    executor.add_node(JetbotTTS_node)

    try:
        # rclpy.spin(JetbotTTS_node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        JetbotTTS_node.get_logger().error('An error occured: {}'.format(e))
        print(e)
    finally:
        JetbotTTS_node.cleanup()
        JetbotTTS_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()