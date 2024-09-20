from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jetbot_riva_voice'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
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
    maintainer='jetbot',
    maintainer_email='jenhungho@outlook.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jetbot_ASR = jetbot_riva_voice.script.Jetbot_ASR_Processor:main',
            'jetbot_TTS = jetbot_riva_voice.script.Jetbot_TTS_Processor:main',
            'jetbot_voice_agent = jetbot_riva_voice.script.Jetbot_ASR_Agent:main',
            'audio_list = jetbot_riva_voice.script:main'
        ],
    },
)
