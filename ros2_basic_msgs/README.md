# Custom ROS2 Interfaces Package

This ROS2 package contains custom message and service definitions to be used within your ROS2 application. It is designed to demonstrate how to create and manage custom interfaces in ROS2.

## Package Structure

After creating the package, the following directories are added:

- `msg/`: Directory for custom message definitions.
- `srv/`: Directory for custom service definitions.

## Getting Started

### Prerequisites

- ROS2 (Rolling, Humble, Foxy, etc.)
- Colcon build tool
- Basic knowledge of ROS2 package creation and management

### Creating the Package

1. Create the ROS2 package:
    ```bash
    ros2 pkg create <your_package_name> --build-type ament_cmake --dependencies rclcpp std_msgs
    ```

2. Navigate to the package directory:
    ```bash
    cd <your_package_name>
    ```

3. Create the `msg` and `srv` directories to store custom message and service files:
    ```bash
    mkdir msg srv
    ```

### Adding Custom Messages and Services

1. Add your custom message files (e.g., `MyMessage.msg`) to the `msg` directory.
2. Add your custom service files (e.g., `MyService.srv`) to the `srv` directory.

### Modifying `package.xml`

Ensure the following modifications are made in your `package.xml`:

1. Add a dependency for `rosidl_default_generators`:
    ```xml
    <build_depend>rosidl_default_generators</build_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```

2. Ensure that your package includes `message_generation` and `message_runtime` dependencies:
    ```xml
    <depend>rclcpp</depend>
    <depend>std_msgs</depend>
    ```



### Modifying `CMakeLists.txt`

Make the following modifications in your `CMakeLists.txt`:

1. Add dependencies for custom messages and services:
    ```cmake
   # find dependencies
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    ```

2. Add the custom message and service files:
    ```cmake
    rosidl_generate_interfaces(${PROJECT_NAME}
    # add custom msgs here  
    "msg/Age.msg"
    
    # add custom srv here
    "srv/AddTwoInts.srv")
    ```

3. Ensure the following lines are present to build and install the package correctly:
    ```cmake
    ament_package()
    ```

### Building the Package

Build the package using `colcon`:

```bash
colcon build --packages-select <your_package_name>
```
### Usage of the Package
Update the package.xml file of the package where you want to use the following custom interfaces. 
```bash
<depend>ros2_basic_msgs</depend>
```

