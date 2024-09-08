from setuptools import find_packages, setup

package_name = 'ros2_basic_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'talker = ros2_basic_python.talker:main',
            'listener = ros2_basic_python.listener:main'
        ],
    },
)
