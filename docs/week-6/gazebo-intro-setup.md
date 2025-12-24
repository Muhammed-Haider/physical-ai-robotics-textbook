# Week 6: Gazebo Simulation: Introduction and Basic Environment Setup

## Learning Outcomes
By the end of this chapter, you will be able to:
1. Understand the purpose and capabilities of Gazebo as a robotic simulator.
2. Be able to install and launch Gazebo in a ROS 2 environment.
3. Learn to create and load simple robot models (URDF).
4. Understand basic Gazebo world files and how to modify them.

## Introduction: Bringing Robots to a Virtual World

(FR-006: Contextual Humanity - *Connecting simulation to practical robotics development*)
In the previous weeks, we established the foundations of Physical AI, explored sensorimotor systems, and delved into the communication backbone of ROS 2. Now, it's time to bring our robots to life – virtually. Building and testing robots in the real world can be expensive, time-consuming, and even dangerous. This is where robotic simulators become invaluable. Gazebo, a powerful 3D robot simulator, allows us to accurately model robots, test algorithms, and experiment with environments in a safe, repeatable, and cost-effective manner. This week, you will learn the fundamentals of Gazebo, from installation to setting up basic simulated environments and loading your first robot models.

## Core Concepts

### 1. What is Gazebo? (FR-005: Modular Mind)

(FR-001: Developmental Staging - *Introducing the concept of simulation in robotics*)
Gazebo is an open-source 3D robot simulator designed for complex indoor and outdoor environments. It provides robust physics engines (like ODE, Bullet, Simbody, DART), high-quality graphics, and convenient interfaces for both users and programs. Its tight integration with ROS 2 makes it the de-facto standard for simulating ROS-enabled robots. Gazebo allows researchers and developers to test algorithms, design robots, and perform regression testing in a virtual world.

**Key Features**:
*   **Physics Engine**: Realistic simulation of gravity, friction, collisions, etc.
*   **Sensors**: Simulated cameras, lidar, IMUs, contact sensors, etc.
*   **Models**: Supports various robot descriptions like URDF (Unified Robot Description Format) and SDF (Simulation Description Format).
*   **Plugins**: Extend Gazebo's functionality with custom behaviors or interactions.

**Analogy**: (FR-004: Creative Synthesis)
Think of Gazebo as a highly detailed video game engine, but instead of simulating a fantasy world for entertainment, it simulates the real physical world for robots. You can create the 'game' environment (a factory, a city street), design the 'characters' (your robots), and then program their 'AI' to interact within that virtual world.

### 2. Gazebo and ROS 2 Integration (FR-005: Modular Mind)

Gazebo's strength in the robotics community largely stems from its seamless integration with ROS 2. ROS 2 provides the communication infrastructure, while Gazebo provides the physics-based virtual world. ROS 2 nodes can publish sensor data from simulated Gazebo sensors, and other ROS 2 nodes can publish commands to simulated Gazebo actuators, creating a complete virtual robot system.

### 3. Robot Models: URDF (Unified Robot Description Format) (FR-001: Developmental Staging)

URDF is an XML format used in ROS to describe all elements of a robot. It defines the robot's kinematic and dynamic properties, visual appearance, and collision geometry. While URDF primarily describes a robot's kinematics (how its joints and links are connected), Gazebo often uses SDF (Simulation Description Format) for more comprehensive world and model descriptions, including additional physics properties. For simplicity, we'll focus on URDF for defining our robot's structure.

## Hands-On Lab: Installing Gazebo and Launching a Sample Robot (FR-002: Constructivist Activity)

### Exercise 1: Gazebo Installation and Basic Launch (FR-003: Motivational Immersion)

**Challenge Level**: Beginner

**Objective**: Install Gazebo and launch a basic ROS 2-integrated simulation to visualize a robot model.

**Tools**: A Linux environment (Ubuntu 22.04 LTS) with ROS 2 Humble installed. (FR-007: Technology Critique - Gazebo is a standard tool in ROS 2 development; hands-on experience is critical for understanding its role in the simulation workflow.)

**Steps**:
1.  **Install Gazebo (if not already installed with ROS 2)**:
    ```bash
    sudo apt update
    sudo apt install gazebo ros-humble-gazebo-ros-pkgs
    ```
    `ros-humble-gazebo-ros-pkgs` provides the necessary bridges for Gazebo to communicate with ROS 2.

2.  **Launch a sample ROS 2 Gazebo simulation**:
    ```bash
    source /opt/ros/humble/setup.bash
    ros2 launch gazebo_ros gazebo.launch.py # Launches Gazebo server and client
    ```
    This command will open the Gazebo simulator. You might see an empty world initially.

3.  **Spawn a simple robot model**: In a new terminal (after sourcing ROS 2 setup):
    ```bash
    source /opt/ros/humble/setup.bash
    ros2 run gazebo_ros spawn_entity.py -entity my_robot -file /opt/ros/humble/share/gazebo_plugins/models/cube/model.urdf -x 0 -y 0 -z 1
    ```
    This spawns a simple cube as a URDF model at (0,0,1) in the Gazebo world.

**Expected Output**: The Gazebo GUI should open, and you should see a grey cube model appear, falling slightly and resting on the ground plane due to gravity.

### Exercise 2: Creating a Simple URDF Model

**Challenge Level**: Beginner

**Objective**: Create a simple URDF file for a two-wheeled robot.

**Tools**: A text editor.

**Steps**:
1.  **Create a new directory** in your ROS 2 workspace for your robot description:
    ```bash
    mkdir -p ~/ros2_ws/src/my_robot_description/urdf
    cd ~/ros2_ws/src/my_robot_description/urdf
    ```

2.  **Create a new file** named `my_robot.urdf` and add the following content:
    ```xml
    <?xml version="1.0"?>
    <robot name="my_robot">
      <link name="base_link">
        <visual>
          <geometry>
            <cylinder length="0.6" radius="0.2"/>
          </geometry>
        </visual>
      </link>

      <link name="right_wheel">
        <visual>
          <geometry>
            <cylinder length="0.1" radius="0.15"/>
          </geometry>
        </visual>
      </link>

      <joint name="base_to_right_wheel" type="continuous">
        <parent link="base_link"/>
        <child link="right_wheel"/>
        <origin xyz="0 -0.25 0" rpy="1.5707 0 0"/>
      </joint>
    </robot>
    ```

**Expected Output**: You have created a URDF file that describes a simple robot with a body and one wheel.

### Exercise 3: Spawning a Custom URDF Model in Gazebo

**Challenge Level**: Intermediate

**Objective**: Launch Gazebo and spawn your custom URDF robot model.

**Tools**: Gazebo, ROS 2, and the URDF file from Exercise 2.

**Steps**:
1.  **Create a launch file** to spawn your robot. Create a `launch` directory in your `my_robot_description` package and add the following file `spawn_robot.launch.py`:
    ```python
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import ExecuteProcess
    from launch_ros.actions import Node

    def generate_launch_description():
        robot_description_path = os.path.join(
            get_package_share_directory('my_robot_description'),
            'urdf',
            'my_robot.urdf'
        )

        return LaunchDescription([
            ExecuteProcess(
                cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
                output='screen'
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'my_robot', '-file', robot_description_path],
                output='screen'
            )
        ])
    ```

2.  **Update `package.xml` and `setup.py`**:
    Add dependencies to `package.xml` and install the launch and urdf directories in `setup.py`.

3.  **Build and launch**:
    ```bash
    cd ~/ros2_ws
    colcon build
    . install/setup.bash
    ros2 launch my_robot_description spawn_robot.launch.py
    ```

**Expected Output**: Gazebo will launch, and your custom robot model (a cylinder with a wheel) will be spawned in the world.

## Creative Challenge: Modifying a Basic World (FR-004: Creative Synthesis)

**Design Task**: Gazebo uses SDF files for world descriptions. Can you find the default `empty.world` file (usually in `/usr/share/gazebo-X/worlds/`) and modify it to add a simple box obstacle? How would you then launch Gazebo with your modified world file?

## Real-World Application: Robot Design and Testing (FR-006: Contextual Humanity)

Before fabricating expensive robot hardware, engineers use Gazebo to design, test, and iterate on robot prototypes. They can quickly experiment with different sensor placements, motor configurations, and kinematic structures. This iterative simulation-based design process drastically reduces development costs and accelerates innovation in robotics, from surgical robots to Mars rovers.

## Technology Deep Dive: SDF vs. URDF (FR-007: Technology Critique)

(FR-001: Developmental Staging - *Clarifying model description formats*)
While URDF is excellent for describing a single robot's kinematic and dynamic properties, SDF (Simulation Description Format) is a more powerful and comprehensive XML format for describing everything in a Gazebo simulation, including robots, static objects (like walls and furniture), and environmental properties (e.g., light sources, physics parameters). For complex simulations, SDF is often preferred as it can describe both the robot and its environment in one file.

## Self-Check Assessment (FR-008: Reflective Assessment)

1.  What are two primary benefits of using a robot simulator like Gazebo during robot development?
2.  Explain the basic function of URDF and how it is typically used within Gazebo.
3.  Which ROS 2 package provides the necessary tools for Gazebo to communicate with the ROS 2 ecosystem?
4.  How would you visually inspect the properties of a spawned model within the Gazebo GUI?

## Before Moving On (FR-008: Reflective Assessment)
- [ ] I can explain the purpose of Gazebo in robotics development.
- [ ] I have successfully installed Gazebo and launched a basic simulation.
- [ ] I have spawned a simple URDF model in Gazebo.
- [ ] I understand the concept of world files in Gazebo.

## Next Steps (FR-001: Developmental Staging)
In the next chapter, we will delve deeper into creating custom robot models with URDF, integrating them into Gazebo, and controlling them using ROS 2.
