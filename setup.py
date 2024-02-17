from setuptools import setup
import os
from glob import glob
from setuptools import find_packages

package_name = 'jetbot_voice'

setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name],
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all laumch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include all param files
        (os.path.join('share', package_name, 'param'), glob('param/*params.yaml')),
        # Include all includ file
        (os.path.join('share', package_name, 'include'), glob('include/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jen-Hung Ho',
    maintainer_email='jetbot@todo.todo',
    description='ROS2 nodes for Jetbot voice',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'jetbot_ASR = jetbot_voice.script.jetbot_ASR_Client:main',
        'jetbot_TTS = jetbot_voice.script.jetbot_TTS:main',
        'voice_copilot = jetbot_voice.script.jetbot_tools_copilot:main',
        'audio_list = jetbot_voice.script.audio_list:main'
        ],
    },
)
