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
import ast    # Parse string into a 2D array
import subprocess
import time

from rclpy.node import Node # Handles the creation of nodes
from threading import Lock
from rcl_interfaces.msg import ParameterType, SetParametersResult, Parameter
from std_msgs.msg import String

from jetbot_riva_voice.include.text_classifier_utility import TextClassifier
from jetbot_riva_voice.include.node_parameter_utility import NodeParamTools

class JetbotASRagent(Node):

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'start' and param.type_ == Parameter.Type.BOOL:
                self.start = param.value
                self.get_logger().info('start= {}'.format(bool(param.value)))

        return SetParametersResult(successful=True)

    def __init__(self):
        super().__init__('Jetbot_ASR_agent')

        self.start = self.declare_parameter('start', True).get_parameter_value().bool_value
        self.ASR_topic = self.declare_parameter('ASR_topic', '/jetbot_voice/transcripts').get_parameter_value().string_value
        self.TTS_topic = self.declare_parameter('TTS_topic', '/chatbot/response').get_parameter_value().string_value
        self.LLM_topic = self.declare_parameter('LLM_topic', '/jetbot_llm_input').get_parameter_value().string_value
        self.LLM_vision_topic = self.declare_parameter('VISION_topic', '/llm_vision_input').get_parameter_value().string_value
        self.ASR_node = self.declare_parameter('ASR_node', '/Riva_ASR_processor').get_parameter_value().string_value
        self.command_nodes = self.declare_parameter('command_nodes', ["/Jetbot_Param_Client"]).get_parameter_value().string_array_value
        self.lable_path = self.declare_parameter('label_path', '/data/models/class_labels.json').get_parameter_value().string_value
        self.model_path = self.declare_parameter('model_path', '/data/models/ASR_classify_model').get_parameter_value().string_value
        self.predict_threshold = self.declare_parameter('predict_threshold', 0.7).get_parameter_value().double_value
        self.command_enable = self.declare_parameter('command_enable', False).get_parameter_value().bool_value
        # Get the parameter as a string (2 dimentional string array)
        self.jetbot_commands = self.declare_parameter('jetbot_commands', "[['start', '0:start'], ['stop', '0:stop']]").get_parameter_value().string_value
        self.jetbot_chat = self.declare_parameter('jetbot_chat', "[['hello', 'hello 1'], ['bye', 'bye 2']]").get_parameter_value().string_value
        self.jetbot_vision = self.declare_parameter('jetbot_vision', "[['vision', 'vision 1']]]").get_parameter_value().string_value


        # YAML debug
        self.get_logger().info('YAML command: {}'.format(self.jetbot_commands))
        self.get_logger().info('YAML chat: {}'.format(self.jetbot_chat))

        # Parse the string into a 2D array
        self.cmd_two_dim_array = ast.literal_eval(self.jetbot_commands)
        self.cmd_dict_array = {row[0]: row[1] for row in self.cmd_two_dim_array}
        self.chat_two_dim_array = ast.literal_eval(self.jetbot_chat)
        self.chat_dict_array = {row[0]: row[1] for row in self.chat_two_dim_array}
        self.vision_two_dim_array = ast.literal_eval(self.jetbot_vision)
        self.vision_dict_array = {row[0]: row[1] for row in self.vision_two_dim_array}

        # Collect command and chat keywords
        self.keywords = []
        command_values =  [row[0] for row in self.cmd_two_dim_array]       
        self.chat_values =  [row[0] for row in self.chat_two_dim_array]
        vision_values = [row[0] for row in self.vision_two_dim_array]
        self.keywords =  self.chat_values + command_values + vision_values

        self.get_logger().info('start            : {}'.format(self.start))
        self.get_logger().info('ASR_topic        : {}'.format(self.ASR_topic))
        self.get_logger().info('TTS_topic        : {}'.format(self.TTS_topic))
        self.get_logger().info('LLM_topic        : {}'.format(self.LLM_topic))
        self.get_logger().info('LLM VISION_topic : {}'.format(self.LLM_vision_topic))
        self.get_logger().info('ASR_node         : {}'.format(self.ASR_node))
        self.get_logger().info('command_nodes    : {}'.format(self.command_nodes))
        self.get_logger().info('jetbot_keywords  : {}'.format(self.keywords))
        self.get_logger().info('predict threshold: {}'.format(self.predict_threshold))
        self.get_logger().info('command enable   : {}'.format(self.command_enable))

        # ASR message keywords -- reference MatchboxNet classes 
        # self.keywords = ["hello", "yes", "no", "up", "down", "left", "right", "on", "off", "unknown", "silence", "start", "stop", "come", "follow", "go"]

        self.lock = Lock()

        # Add parameters callback 
        self.add_on_set_parameters_callback(self.parameter_callback)

        # self.init_ros_nodes()
        self.node_param_util = NodeParamTools(self, executor)

        # Create the subscriber. This subscriber will receive lidar message
        self.subscription = self.create_subscription(
            String, 
            self.ASR_topic,
            self.ASR_callback,
            10)
        
        self.pub_TTS = self.create_publisher(
            String,
            self.TTS_topic,
            10
        )

        self.pub_LLM = self.create_publisher(
            String,
            self.LLM_topic,
            10
        )

        self.pub_LLM_vision = self.create_publisher(
            String,
            self.LLM_vision_topic,
            10
        )


        # Load robot command and load tensorflow model data
        self.init_text_classifier()

    #
    # Remove nodes for get/set parameter service call
    #
    def cleanup(self):
        # clean up set_param_node, get_param_node
        self.node_param_util.cleanup()
        pass

    #
    # Load robot command and load tensorflow model data
    #
    def init_text_classifier(self):
        self.classifier = TextClassifier(self.model_path, self.lable_path)
        self.get_logger().info("===================================================")
        self.get_logger().info("Robot commands: {}".format(self.classifier.class_labels))
        self.get_logger().info("===================================================")

    #
    # This function handles the prediction process for the text classifier model
    # 1D convolutional neural network (CNN) model
    #
    def handle_prediction(self, min_score, prediction):
        self.get_logger().info("ASR raw:[{}]".format(prediction))
        label, result, score=  self.classifier.predict(min_score, prediction)
        if result:
            self.get_logger().info("predict: [{}]:[{}]".format(label, score))
        else:
            self.get_logger().info("Chat topic : ================================")
            self.get_logger().info("predict: [{}]:[{}]".format(label, score))

        return result, label, score

    #
    # NVIDIA jetson-voice ASR ROS2 topic subscription
    #
    def ASR_callback(self, msg):
        self.get_logger().info('Raw ASR: [{}]:{}'.format(len((msg.data).split()), msg.data))

        greeting = False
        if len((msg.data).split()) == 1:
            # Greeting is static chat - response define in jetbot_chat: 2 dimention array
            self.get_logger().info('static chat keyword list:{}'.format(self.chat_values))
            greeting, keyword = self.filter_keywords(msg.data, self.chat_values)
            self.get_logger().info('greeting: {}:{}'.format(greeting, keyword))

        # If the input is not a greeting, utilize the 1D convolutional neural network (CNN) model 
        # for text classification to determine the user's intention for JetBot.
        if greeting == False:
            # Filter out ASR noise -- 1D convolutional neural network (CNN) model
            result, label, score = self.handle_prediction(self.predict_threshold, msg.data)
            # Only pick up ASR input contains jetbot command keywords
            found, keyword = self.filter_keywords(label, self.keywords)
        else:
            found = greeting

        command = False
        vision_chat = False
        chat = False

        ASR_string = msg.data
        node_name = "/jetbot"

        if found:
            if greeting and keyword in self.chat_dict_array:
                ASR_string = self.chat_dict_array[keyword]
                self.get_logger().info("ASR input: {} greeting: {}".format(keyword,ASR_string))
            elif keyword in self.vision_dict_array:
                # ASR_string = self.chat_dict_array[keyword]
                self.get_logger().info("ASR input: {} chat: {}".format(keyword,ASR_string))
                vision_chat = True
            elif keyword in self.cmd_dict_array:
                command = True
                self.get_logger().info('jetbot command tool enable: {}'.format(command))
                # Retrieve node name and command value [index:value]
                parts = self.cmd_dict_array[keyword].split(':')
                node_index = int(parts[0])
                if node_index < len(self.command_nodes):
                    node_name = self.command_nodes[node_index]
                else:
                    self.get_lobber().info('Error: incorrect node name index:{}'.format(node_index))
                # Retrive command parameter
                ASR_string = parts[1]

                # Turn on command feature if jetbot tools copilot node exist
                if self.command_enable == False:
                    node_exist = self.check_node_exists(node_name)
                    self.get_logger().info('check node:{} exit:{}'.format(node_name, node_exist))
                    if node_exist:
                        self.get_logger().info('Turn on node:{} start parameter'.format(node_name))
                        self.enable_jetbot_tool_copilot(node_name)
                        self.command_enable = True
                        time.sleep(1.0)

        else:
            self.get_logger().info("ASR input not found in keyword list --> chatbot:" + msg.data)
            chat = True
            node_name = self.ASR_node
            # return

        # Block the next callback from executing until the current callback finishes 
        with self.lock:
            # Jetbot chat acton no need to set command to target node
            if command == True:
                # 'Echoing' in ASR occurs
                # when the microphone picks up the system's own text-to-speech output,
                # creating a recursive voice recognition loop.
                self.mute_ASR_processor(self.ASR_node)
                if self.command_enable:
                    passfail = self.node_param_util.try_set_node_parameters(node_name, 'command', type=ParameterType.PARAMETER_STRING, value=ASR_string)
                    if passfail == True:
                        ASR_string = "jetbot process: " + ASR_string
                    else:
                        ASR_string = "jetbot node :{} not exit skip command :{}" + ASR_string

                    '''
                    value = self.node_param_util.get_node_parameters(node_name, 'command')
                    if value.string_value != ASR_string:
                        # Set voice command parameter to target node
                        self.node_param_util.set_node_parameters(node_name, 'command', type=ParameterType.PARAMETER_STRING, value=ASR_string)
                        ASR_string = "jetbot process: " + ASR_string
                    else:
                        # 'Echoing' in ASR occurs
                        # when the microphone picks up the system's own text-to-speech output,
                        # creating a recursive voice recognition loop.

                        # Ignore command value alread set, 
                        # Wait until Jetbot_tool_voice_copilot complete the command and reset the value
                        self.get_logger().info('ASR input command alread set --> ignore:' + ASR_string)
                        return
                    '''
                else:
                    # Publish to TTS node to play audio streaming and disable ASR muting
                    ASR_string = "jetbot tool copilot command: " + ASR_string
                TTS_string = String()
                TTS_string.data = ASR_string
                self.pub_TTS.publish(TTS_string)
            elif greeting == True:
                self.mute_ASR_processor(self.ASR_node)
                TTS_string = String()
                TTS_string.data = ASR_string
                self.pub_TTS.publish(TTS_string)
            elif vision_chat == True:
                self.mute_ASR_processor(self.ASR_node)
                # Publish to LLM vidion node to response as chatbot
                LLM_vision_string = String()
                # Use ASR raw input data as LLM input
                LLM_vision_string.data = msg.data
                self.pub_LLM_vision.publish(LLM_vision_string)
            elif chat == True:
                self.mute_ASR_processor(self.ASR_node)
                # Publish to LLM node to response as chatbot
                LLM_string = String()
                # Use ASR raw input data as LLM input
                LLM_string.data = msg.data
                self.pub_LLM.publish(LLM_string)

    #
    # Mute ASR processor and wait until LLM chat reponse to TTS task complete
    #
    def mute_ASR_processor(self, node_name):
        # Turn off ASR and wait until LLM chat response to TTS task complete
        self.set_jetbot_node_bool_parameters(node_name, 'start', False)

    #
    # Enable Jetbot voice 
    #
    def enable_jetbot_tool_copilot(self, node_name):
        # Trun on Jerbot voice 
        self.set_jetbot_node_bool_parameters(node_name, 'start', True)

    #
    # Set jetbot ROS2 node bool parameter
    #
    def set_jetbot_node_bool_parameters(self, node_name, parameter, bool_value):
        passfail, value = self.node_param_util.try_get_node_parameters(node_name, parameter)
        if passfail == True:
            self.get_logger().info('Jetbot node:{} param:{} value:{}'.format(node_name, parameter, value.bool_value))
            if value.bool_value != bool_value:
                self.node_param_util.try_set_node_parameters(node_name, parameter, type=ParameterType.PARAMETER_BOOL, value=bool_value)
        else:
            self.get_logger().info('Jetbot chat {} node not exit, skip the task'.format(node_name))

    #
    # Filter out ASR noise -- TODO how to improve the filtering 
    # Only pick up ASR input contains keywords
    #
    def filter_keywords(self, asr_output, keywords):
        # Convert ASR output and keywords to lowercase for case-insensitive matching
        asr_output = asr_output.lower()
        keywords = [keyword.lower() for keyword in keywords]

        # Find keywords in ASR output
        for keyword in keywords:
            if keyword in asr_output:
                return True, keyword

        # If no keyword found, return False and None
        return False, None

    #
    # check if a node exist in ROS2
    #
    def check_node_exists(self, node_name):
        result = subprocess.run(['ros2', 'node', 'list'], stdout=subprocess.PIPE)
        nodes = result.stdout.decode().split('\n')
        return node_name in nodes



def main(args=None):

    rclpy.init(args=args)

    global executor

    executor = rclpy.executors.MultiThreadedExecutor()

    JetbotASR_node = JetbotASRagent()
    executor.add_node(JetbotASR_node)

    try:
        # rclpy.spin(JetbotTTS_node)
        executor.spin()
    except KeyboardInterrupt:
        print('\ncontrol-c: JetbotTTS_node shutting down')
    finally:
        # Destroy the node explictly - don't depend on garbage collector
        JetbotASR_node.cleanup()
        JetbotASR_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
