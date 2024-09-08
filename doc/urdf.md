# URDF File Overview

URDF (Universal Robot Description Format) is used to define the structure and properties of robots. It uses XML syntax to describe various aspects of a robot including:

- **Links**: The individual components of the robot.
- **Joints**: The connections between links that allow movement.
- **Shapes and Meshes**: Visual representations of the robot components.
- **Sensors**: Devices used for perception.
- **Actuators**: Components that control movement.
- **Physical Properties**: Parameters for simulations such as mass and inertia.

## What is a LINK?

In URDF, a **link** represents an individual unit or component of the robot. Each link can be defined with specific properties including its visual appearance and physical characteristics.

### Example: Defining a Link

The following example shows a URDF file defining a link named `base_link`. This link is visualized as a simple box with dimensions of 0.1 meters on each side.

```xml
<?xml version="1.0"?>
<robot name="urdfbot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

### Example: Defining a Joints

Joints connect two links and allow relative movement between them. URDF supports various types of joints:

- **Revolute Joint**: Allows rotation around a single axis.
- **Prismatic Joint**: Allows linear movement along a single axis.
- **Fixed Joint**: No relative movement; the links are rigidly connected.
- **Continuous Joint**: Allows continuous rotation, similar to revolute but with no limits.

```xml
<joint name="joint_1" type="revolute">
  <parent link="base_link"/>
  <child link="link_1"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit effort="100" velocity="1.0" lower="-1.57" upper="1.57"/>
</joint>
```

### Gazebo Plugins
Gazebo plugins extend the functionality of URDF by integrating with the Gazebo simulation environment. Plugins can be used to add sensors, controllers, and custom behaviors.

Example: Adding a Gazebo Sensor Plugin
```
<gazebo>
  <sensor name="camera" type="camera">
    <pose>0 0 1 0 0 0</pose>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <update_rate>30.0</update_rate>
  </sensor>
</gazebo>
```


# URDF Geometries and Units of Measurement

URDF (Universal Robot Description Format) supports several basic geometries for defining links. Below is a summary of the available geometries and the units of measurement used in URDF files.

## Basic Geometries for Defining Links

URDF supports the following basic geometries:

- **Box**: Defined by its size (length, width, height).
  ```xml
  <box size="x y z"/>
  ```
- **Cylinder**: Defined by its radius and length.
  ```xml
  <cylinder radius="r" length="l"/>
  ```
- **Sphere**: Defined by its radius.
  ```xml
  <sphere radius="r"/>
  ```

## Units of Measurement

URDF files use the International System of Units (SI) for measurements:

- **Distance**: meters
- **Angles**: radians
- **Weight**: kilograms

For instance, in the provided example, the box is defined with dimensions of 0.1 meters on all sides.

## Visualization

To visualize the URDF file with the `base_link` or any other defined links, you can use tools that support URDF visualization. Make sure to select XML syntax in your IDE for improved readability and editing.

