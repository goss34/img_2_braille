from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'braille_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f"{package_name}.util"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoonlabbraille',
    maintainer_email='yoonlabbraille@todo.todo',
    description='ROS 2 package for interfacing with Canute 360 Braille display.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heightmap_generator_node = braille_interface.heightmap_generator_node:main',
            'braille_server_node = braille_interface.braille_server_node:main',
            'publish_map_node = braille_interface.publish_map_node:main',
            'mock_map_node = braille_interface.mock_map_node:main',
            'image_to_braille_node = braille_interface.image_to_braille_node:main',
        ],
    },
)