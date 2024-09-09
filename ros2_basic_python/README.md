# ROS2 Basic Python

## Creating a ROS2 Package

To create your package, use the command 
```
ros2 pkg create --build-type ament_python <package_name> --dependencies rclpy
```
The <package_name> is the name of the package you want to create, and the <package_dependency_X> are
the names of other ROS2 packages that your package depends on. Note also that we are specifying ament_python as the build type . This indicates that we are creating a Python package.It is a good idea to build your package after it has been created. It is the quickest way to determine if the dependencies you listed can be resolved and check that there are no mistakes in the entered data.

```
cd ~/ros2_ws/
colcon build
source install/setup.bash
```

### Modifying the setup.py File
The setup.py file contains all the necessary instructions for properly compiling your package. The main objective of this code is to generate an executable for the scripts we will make.  

For Scripting 
```
import os
from glob import glob
from setuptools import setup
package_name = 'my_package'
setup(
#code
...
#code
    entry_points={
        'console_scripts': [
            'talker_node = ros2_basic_python.talker:main',
            'listener_node = ros2_basic_python.listener:main',
            'custom_msg_node = ros2_basic_python.age:main'
        ],
    }
#code
...
)

```
format:  '<executable_name> = <package_name>.<script_name>:main'

For Launch File 
```
import os
from glob import glob

from setuptools import setup
package_name = 'my_package'
setup(
#code
...
#code
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
#code
...
#code
)
```
The objective of this code is to install the launch files. For example, with the package named my_package , this will install all the launch files from the launch/ folder, into
```
~/ros2_ws/install/my_package/share/my_package/ .
```
