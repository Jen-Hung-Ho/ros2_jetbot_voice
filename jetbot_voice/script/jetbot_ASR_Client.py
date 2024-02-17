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

from rclpy.node import Node # Handles the creation of nodes
from threading import Lock
from rcl_interfaces.msg import ParameterType, SetParametersResult, Parameter
from std_msgs.msg import String

from jetbot_tools.include.node_parameter_utility import NodeParamTools

class JetbotASRclient(Node):

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'start' and param.type_ == Parameter.Type.BOOL:
                self.start = param.value
                self.get_logger().info('start= {}'.format(bool(param.value)))

        return SetParametersResult(successful=True)

    def __init__(self):
        super().__init__('Jetbot_ASR_client')

        self.start = self.declare_parameter('start', True).get_parameter_value().bool_value
        self.ASR_topic = self.declare_parameter('ASR_topic', '/voice/transcripts').get_parameter_value().string_value
        self.TTS_topic = self.declare_parameter('TTS_topic', '/TTS/transcripts').get_parameter_value().string_value
        self.command_nodes = self.declare_parameter('command_nodes', ["/Jetbot_Param_Client"]).get_parameter_value().string_array_value
        # Get the parameter as a string (2 dimentional string array)
        self.jetbot_commands = self.declare_parameter('jetbot_commands', ["hello", "yes", "no"]).get_parameter_value().string_value
        self.jetbot_chat = self.declare_parameter('jetbot_chat', "[['hello', 'hello 1'], ['bye', 'bye 2']]").get_parameter_value().string_value

        # YAML debug
        self.get_logger().debug('YAML command: {}'.format(self.jetbot_commands))
        self.get_logger().debug('YAML chat: {}'.format(self.jetbot_chat))

        # Parse the string into a 2D array
        self.cmd_two_dim_array = ast.literal_eval(self.jetbot_commands)
        self.cmd_dict_array = {row[0]: row[1] for row in self.cmd_two_dim_array}
        self.chat_two_dim_array = ast.literal_eval(self.jetbot_chat)
        self.chat_dict_array = {row[0]: row[1] for row in self.chat_two_dim_array}

        # Collect command and chat keywords
        self.keywords = []
        command_valuse =  [row[0] for row in self.cmd_two_dim_array]       
        chat_valuse =  [row[0] for row in self.chat_two_dim_array]
        self.keywords =  chat_valuse + command_valuse

        self.get_logger().info('start : {}'.format(self.start))
        self.get_logger().info('ASR_topic    : {}'.format(self.ASR_topic))
        self.get_logger().info('TTS_topic    : {}'.format(self.TTS_topic))
        self.get_logger().info('command_nodes: {}'.format(self.command_nodes))
        self.get_logger().info('jetbot_keywords: {}'.format(self.keywords))

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

    #
    # Remove nodes for get/set parameter service call
    #
    def cleanup(self):
        # clean up set_param_node, get_param_node
        self.node_param_util.cleanup()
        pass


    #
    # NVIDIA jetson-voice ASR ROS2 topic subscriptio
    #
    def ASR_callback(self, msg):
        self.get_logger().info('Raw ASR: "%s"' % msg.data)

        # Filter out ASR noise -- TODO how to improve the filtering 
        # Only pick up ASR input contains keywords
        found, keyword = self.filter_keywords(msg.data, self.keywords)

        command = False
        ASR_string = msg.data
        node_name = "/jetbot"

        if found:
            if keyword in self.chat_dict_array:
                ASR_string = self.chat_dict_array[keyword]
                self.get_logger().info("ASR input: {} chat: {}".format(keyword,ASR_string))
            elif keyword in self.cmd_dict_array:
                command = True
                # Retrieve node name and command value [index:value]
                parts = self.cmd_dict_array[keyword].split(':')
                node_index = int(parts[0])
                if node_index < len(self.command_nodes):
                    node_name = self.command_nodes[node_index]
                else:
                    self.get_lobber().info('Error: incorrect node name index:{}'.format(node_index))
                ASR_string = parts[1]
        else:
            self.get_logger().info("ASR input not found in keyword list --> ignore:" + msg.data)
            return

        # Block the next callback from executing until the current callback finishes 
        with self.lock:
            # Jetbot chat acton no need to set command to target node
            if command == True:
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
                    self.get_logger().info("ASR input command alread set --> ignore:" + ASR_string)
                    return 

            # Publish to TTS node to play audio streaming
            TTS_string = String()
            TTS_string.data = ASR_string
            self.pub_TTS.publish(TTS_string)


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


def main(args=None):

    rclpy.init(args=args)

    global executor

    executor = rclpy.executors.MultiThreadedExecutor()

    JetbotASR_node = JetbotASRclient()
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