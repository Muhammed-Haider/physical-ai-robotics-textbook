# Week 7: Gazebo & Unity Simulation: Advanced Models, Sensors, and ROS 2 Integration

## Learning Outcomes
By the end of this chapter, you will be able to:
1. Learn to create and integrate complex robot models (URDF/SDF) in Gazebo.
2. Understand how to add and configure various simulated sensors (e.g., cameras, LiDAR) in Gazebo.
3. Integrate ROS 2 control interfaces with simulated robots in Gazebo.
4. Explore the basics of Unity for high-fidelity robotics simulation.

## Introduction: Mastering the Virtual Robotics Lab

(FR-006: Contextual Humanity - *Connecting advanced simulation to real-world robot development cycles*)
In Week 6, we took our first steps into the virtual world of Gazebo, understanding its purpose and launching simple robot models. This week, we elevate our simulation skills. Building a truly autonomous robot requires more than just basic models; it demands sophisticated sensor data, accurate physical interactions, and seamless integration with our ROS 2 control architecture. We will delve into creating and refining complex robot descriptions, adding realistic sensors, and establishing a robust ROS 2 bridge within Gazebo. Furthermore, we'll get a first taste of Unity, a powerful game engine that is increasingly being adopted for high-fidelity robotics simulation, offering unparalleled visual realism and advanced rendering capabilities for specialized tasks.

## Core Concepts

### 1. Advanced Robot Modeling with URDF and SDF (FR-005: Modular Mind)

(FR-001: Developmental Staging - *Building on basic URDF knowledge from Week 6*)
While simple URDF models are great for introductions, complex robots require more detailed descriptions. We'll explore advanced URDF features for joints, transmissions, and robot components. Crucially, we'll also dive deeper into SDF (Simulation Description Format), which provides a more comprehensive way to describe not just robots, but entire simulation environments, including physics properties, lights, and static objects, all within a single file. Understanding the nuances of both URDF (for ROS compatibility) and SDF (for Gazebo's full capabilities) is key.

**Analogy**: (FR-004: Creative Synthesis)
If a simple URDF is a basic blueprint of a house showing its rooms, an advanced URDF might include details of the plumbing and electrical systems. An SDF, on the other hand, is like the entire architectural plan, including the landscape, lighting, and even the type of soil the house is built on.

### 2. Simulated Sensors: Bringing Perception to the Virtual Robot (FR-005: Modular Mind)

(FR-001: Developmental Staging - *Extending sensor concepts from Week 2 to simulation*)
A robot is only as good as its perception. In Gazebo, we can add a variety of simulated sensors that mimic real-world counterparts.
*   **Cameras**: RGB, depth, and stereo cameras provide visual data. We configure their resolution, field of view, and noise properties.
*   **Lidar**: Simulated laser scanners produce point cloud data for environmental mapping and obstacle detection.
*   **IMUs**: Provide orientation and acceleration data for robot localization and control.
*   **Contact Sensors**: Detect physical collisions.

Adding these sensors to your robot's model (via URDF/SDF extensions) and configuring them to publish ROS 2 messages is fundamental for developing perception algorithms without real hardware.

### 3. ROS 2 Control with Simulated Robots (FR-005: Modular Mind)

(FR-001: Developmental Staging - *Integrating ROS 2 control from Weeks 3-5 with simulation*)
The ultimate goal of simulation is to test our ROS 2 control algorithms. This involves:
*   **ros2_control**: A powerful ROS 2 package that provides a generic framework for robot control, interfacing with hardware or simulation.
*   **Joint State Publishers**: Nodes that publish the current state of a robot's joints.
*   **Controllers**: ROS 2 nodes that take desired commands (e.g., joint positions, velocities) and send them to the simulated robot's actuators in Gazebo.

Seamless integration allows us to develop and debug complex robot behaviors entirely within the simulation, ready for deployment on physical hardware.

### 4. Introduction to Unity for Robotics Simulation (FR-001: Developmental Staging)

While Gazebo is powerful, Unity (a popular game development engine) offers unparalleled visual fidelity, advanced rendering, and sophisticated physics. It's becoming a strong contender for high-fidelity robotics simulation, particularly for tasks requiring realistic rendering, human-robot interaction visualization, or complex sensor simulations (e.g., advanced camera effects, lighting). Unity's strength lies in its graphical capabilities and a vast asset store. We'll explore basic concepts of setting up a Unity project for robotics and integrating it with ROS 2 through packages like `ROS-TCP-Connector`.

## Hands-On Lab: Custom Robot Model with Sensors and ROS 2 Control in Gazebo (FR-002: Constructivist Activity)

### Exercise 1: Building a Simple ROS 2 Robot in Gazebo (FR-003: Motivational Immersion)

**Challenge Level**: Intermediate

**Objective**: Create a custom URDF model for a simple mobile robot, add a simulated LIDAR sensor, and integrate it with ROS 2 for basic control.

**Tools**: ROS 2 development environment, Gazebo. (FR-007: Technology Critique - This exercise combines previous knowledge to build a more functional simulated robot, demonstrating end-to-end integration essential for real-world development.)

**Steps**:
1.  **Create a custom URDF file for a simple wheeled robot (`my_robot.urdf`)**: Define its base, wheels, and a placeholder for a LiDAR sensor.
    ```xml
    <?xml version="1.0"?>
    <robot name="my_wheeled_robot">
      <link name="base_link">
        <visual>
          <geometry>
            <box size="0.2 0.2 0.1"/>
          </geometry>
        </visual>
        <collision>
          <geometry>
            <box size="0.2 0.2 0.1"/>
          </geometry>
        </collision>
      </link>
      <!-- Add wheels as links and joints -->
      <!-- Add a simple LiDAR sensor using a Gazebo reference and plugin -->
      <gazebo reference="lidar_link">
        <sensor name="laser_sensor" type="ray">
          <pose>0 0 0 0 0 0</pose>
          <visualize>true</visualize>
          <update_rate>10</update_rate>
          <ray>
            <scan>
              <horizontal>
                <samples>360</samples>
                <resolution>1</resolution>
                <min_angle>-3.14</min_angle>
                <max_angle>3.14</max_angle>
              </horizontal>
            </scan>
            <range>
              <min>0.1</min>
              <max>10.0</max>
              <resolution>0.01</resolution>
            </range>
          </ray>
          <plugin name="ros_ray_sensor_controller" filename="libgazebo_ros_ray_sensor.so">
            <ros>
              <namespace>/my_robot</namespace>
              <argument>--ros-args -r __ns:=/my_robot</argument>
              <remapping>~/out:=scan</remapping>
            </ros>
            <output_type>sensor_msgs/LaserScan</output_type>
            <topicName>scan</topicName>
            <frameName>lidar_link</frameName>
          </plugin>
        </sensor>
      </gazebo>
    </robot>
    ```

2.  **Create a ROS 2 Launch file to spawn the robot and start controllers**:
    ```python
    # my_robot_launch.py
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch_ros.actions import Node
    from launch.actions import ExecuteProcess

    def generate_launch_description():
        my_package_dir = get_package_share_directory('my_robot_description') # Assuming your URDF is in a package called 'my_robot_description'
        urdf_file_path = os.path.join(my_package_dir, 'urdf', 'my_robot.urdf')

        return LaunchDescription([
            ExecuteProcess(
                cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
                output='screen'
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': urdf_file_path, 'use_sim_time': True}]
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'my_robot', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '1'],
                output='screen'
            )
            # Add controllers if ros2_control setup
        ])
    ```
3.  **Build your workspace** and **source setup files**.
4.  **Launch the simulation**: `ros2 launch my_robot_description my_robot_launch.py`

**Expected Output**: Gazebo will launch, and your custom robot model with a simulated LiDAR sensor will appear. You should be able to visualize the LiDAR scans in RViz2 by subscribing to the `/my_robot/scan` topic.

## Creative Challenge: Unity High-Fidelity Simulation (FR-004: Creative Synthesis)

**Design Task**: Research and briefly outline the steps you would take to import your URDF robot model from Gazebo into a Unity project. What advantages would Unity offer for a specific simulation scenario (e.g., detailed human-robot interaction visualization, advanced lighting/textures)?

## Real-World Application: Autonomous Drone Navigation (FR-006: Contextual Humanity)

Simulated drone environments are critical for developing autonomous flight algorithms. Gazebo is often used for physics-accurate flight dynamics and sensor data generation. For testing human interaction with drones or visualizing complex urban environments with high realism, Unity could be employed. The ability to simulate across different platforms allows for a comprehensive testing cycle before real-world deployment.

## Technology Deep Dive: State Estimation with EKF/UKF (FR-007: Technology Critique)

(FR-001: Developmental Staging - *Introducing advanced sensor fusion concepts*)
When integrating multiple simulated sensors (e.g., LiDAR, IMU, camera), their data needs to be fused to get a robust estimate of the robot's state (position, velocity, orientation). Extended Kalman Filters (EKF) and Unscented Kalman Filters (UKF) are common algorithms for this sensor fusion, compensating for sensor noise and providing a more accurate and stable state estimation, critical for precise navigation and manipulation.

## Self-Check Assessment (FR-008: Reflective Assessment)

1.  What is the primary difference in purpose between URDF and SDF when modeling a robot for Gazebo?
2.  List three types of simulated sensors you can add to a robot model in Gazebo.
3.  Why is `ros2_control` an important framework for integrating ROS 2 with simulated robot actuators?
4.  In what specific scenarios might Unity be preferred over Gazebo for robotics simulation?

## Before Moving On (FR-008: Reflective Assessment)
- [ ] I can describe the roles of URDF and SDF in robot modeling.
- [ ] I have integrated a simulated sensor into a robot model.
- [ ] I understand the basics of ROS 2 control integration with Gazebo.
- [ ] I can explain the potential benefits of using Unity for robotics simulation.

## Next Steps (FR-001: Developmental Staging)
In the upcoming chapters, we will transition to the NVIDIA Isaac Platform, exploring its advanced simulation capabilities and specialized tools for accelerated robotics development and AI training.
