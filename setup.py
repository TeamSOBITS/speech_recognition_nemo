import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'speech_recognition_nemo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob('launch/*')),
        (os.path.join('share', package_name, "sound_file"), glob('sound_file/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sobits',
    maintainer_email='sobits@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            "model_download = speech_recognition_nemo.model_download:main",
            "nemo_server = speech_recognition_nemo.nemo_server:main",
            "nemo_client = speech_recognition_nemo.nemo_client:main",
        ],
    },
)
