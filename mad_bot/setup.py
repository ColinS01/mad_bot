import os
from glob import glob
from setuptools import setup

package_name = 'mad_bot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your_email@example.com',
    description='Mad Bot ROS 2 package',
    license='Apache 2.0',
    tests_require=['pytest'],
    data_files=[
                (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
                (os.path.join('share', package_name, 'world'), glob(os.path.join('world', '*.sdf'))),
                (os.path.join('share', package_name, 'model'), glob(os.path.join('model', '*.sdf'))),
                (os.path.join('share', package_name, 'model'), glob(os.path.join('model', '*.urdf'))),
                (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
                (os.path.join('share', package_name, 'image'), glob(os.path.join('image', '*.jpg')))


    ],
    entry_points={
        'console_scripts': [
            'movement_controller = mad_bot.movement_controller:main',
            'camera_controller = mad_bot.camera_controller:main',
        ],
    },
)
