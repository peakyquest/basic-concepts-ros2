from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_basic_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haroon',
    maintainer_email='haroon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker_node = ros2_basic_python.talker:main',
            'listener_node = ros2_basic_python.listener:main',
            'custom_msg_node = ros2_basic_python.age:main',
            'server = ros2_basic_python.add_two_ints_server:main',
            'client = ros2_basic_python.add_two_ints_client:main'
        ],
    },
)
