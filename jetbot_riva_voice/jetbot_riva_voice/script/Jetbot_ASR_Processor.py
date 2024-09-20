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
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType, SetParametersResult
from std_msgs.msg import String

import pyaudio
import riva.client
import riva.client.audio_io

class riva_asr_processor(Node):

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

        self.ASR_topic = self.declare_parameter('ASR_topic', "/voice/transcripts").get_parameter_value().string_value 
        self.RIVA_URL = self.declare_parameter('url', "localhost:50051").get_parameter_value().string_value
        self.index = self.declare_parameter('index', 26).get_parameter_value().integer_value
        self.streaming_chunk = self.declare_parameter('streaming_chunk', 16000).get_parameter_value().integer_value
        self.start = self.declare_parameter('start', True).get_parameter_value().bool_value

        self.get_logger().info("=========================================")
        self.get_logger().info("Jetbot ASR processor :{} start".format(name))
        self.get_logger().info("ASR_topic      : {}".format(self.ASR_topic))
        self.get_logger().info("RIVA url        : {}".format(self.RIVA_URL))
        self.get_logger().info("index           : {}".format(self.index))
        self.get_logger().info("streaming chunk : {}".format(self.streaming_chunk))
        self.get_logger().info("start           : {}".format(self.start))
        self.get_logger().info("=========================================")

        self.publisher = self.create_publisher(String, self.ASR_topic, 10)

        self.msg = String()
        self.i = 0

        # Add parameters callback 
        self.add_on_set_parameters_callback(self.parameter_callback)

        # self.p = pyaudio.PyAudio()
        # self.list_audio_devices()
        self.riva_init()

        timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        self.thread = threading.Thread(target=self.ASR_processor)
        self.thread.start()

    def riva_init(self):
        self.p = pyaudio.PyAudio()
        default_device_info = riva.client.audio_io.get_default_input_device_info()
        self.get_logger().debug("Rivai default info:{}".format(default_device_info))
        # default_index = None if default_device_info is None else default_device_info['index']
        if default_device_info is not None and int(default_device_info['maxInputChannels']) > 0:
            self.audio_index = default_device_info['index']
            self.get_logger().info("use default - ignore user input")
        else:
            self.audio_index = self.index
        default_device = self.p.get_device_info_by_index(self.audio_index)
        self.sample_rate = int(default_device_info['defaultSampleRate'])

        self.get_logger().info("==============================================")
        self.get_logger().info("Audio default index     : {}".format(self.audio_index))
        self.get_logger().info("Max input ouput channels: [{} - {}]".format(default_device['maxInputChannels'], default_device_info['maxOutputChannels']))
        self.get_logger().info("sample rate             : {}".format(self.sample_rate))
        # riva.client.audio_io.list_input_devices()
        self.get_logger().info("==============================================")

    def list_audio_devices(self):
        self.get_logger().info("==============================================")
        self.get_logger().info(" AUDIO DEVICES: ")
        self.get_logger().info("==============================================")
        for i in range(self.p.get_device_count()):
            dev = self.p.get_device_info_by_index(i)
            self.get_logger().info(f"{dev['index']:2d}: {dev['name']:50s} (inputs={dev['maxInputChannels']:<3d} outputs={dev['maxOutputChannels']:<3d} sampleRate={int(dev['defaultSampleRate'])})")
        self.get_logger().info("==============================================")


    def timer_callback(self):
        self.msg.data = 'Hello World: %d' % self.i
        self.i += 1
        self.get_logger().info('Publishing: "%s"' % self.msg.data)
        self.publisher.publish(self.msg)

    # thread: Jetbot ASR processor
    def ASR_processor(self):
        self.get_logger().info('==============================')
        self.get_logger().info('Jetbot ASR processor --> START')
        self.get_logger().info('==============================')

        # Initialize RIVA
        # auth = riva.client.Auth(args.ssl_cert, args.use_ssl, args.server, args.metadata)
        auth = riva.client.Auth(None, False, self.RIVA_URL, None)
        asr_service = riva.client.ASRService(auth)

        config = riva.client.StreamingRecognitionConfig(
            config=riva.client.RecognitionConfig(
                encoding=riva.client.AudioEncoding.LINEAR_PCM,
                language_code="en-US",
                model="",
                max_alternatives=1,
                profanity_filter=False,
                enable_automatic_punctuation=False,
                verbatim_transcripts=True,
                sample_rate_hertz=self.sample_rate,
                audio_channel_count=1,
            ),
            interim_results=True,
        )
        
        boosted_words = ["jetbot", "action"]
        boosted_score = 4.0
        riva.client.add_word_boosting_to_config(config, boosted_words, boosted_score)
        
        if hasattr(riva.client, 'add_endpoint_parameters_to_config'):
            riva.client.add_endpoint_parameters_to_config(
                config,
                -1,  #start history
                -1.0, #start threshold
                -1, # stop history
                -1, # stop history eou
                -1, # stop threshold
                -1.0 # top threshold  eou
            )
        else:
            self.get_logger().info("The function add_endpoint_parameters_to_config() does not exist in this version of RIVA client API.")

        with riva.client.audio_io.MicrophoneStream(
            self.sample_rate,
            self.streaming_chunk,
            device=self.audio_index,
        ) as audio_chunk_iterator:
            responses=asr_service.streaming_response_generator(
                    audio_chunks=audio_chunk_iterator,
                    streaming_config=config
            )

            for response in responses:
                if not response.results:
                    continue
                for result in response.results:
                    if not result.alternatives:
                        continue
                    transcript = result.alternatives[0].transcript
                    if result.is_final:
                        self.get_logger().info('ASR buffer: [ {} ]'.format(transcript))
                        self.get_logger().debug('ASR RAW:{}'.format(result.alternatives))
                        for i, alternative in enumerate(result.alternatives):
                            asr_msg = (f'(alternative {i + 1})' if i > 0 else '') + f' {alternative.transcript}'
                            self.get_logger().info( '## {}'.format(asr_msg))

                        if (self.start):
                            self.msg.data = transcript
                            self.get_logger().info('Publishing: "%s"' % self.msg.data)
                            self.publisher.publish(self.msg)
                        else:
                            self.get_logger().info('ASR -off- ignore: {}'.format(transcript))

        self.get_logger().info('==============================')
        self.get_logger().info('Jetbot ASR processor -- EXIT')
        self.get_logger().info('==============================')



def main(args=None):
    rclpy.init(args=None)

    node = riva_asr_processor('Riva_ASR_processor')

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        node.get_logger().error('An error occured: {}'.format(e))
        print(e)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()