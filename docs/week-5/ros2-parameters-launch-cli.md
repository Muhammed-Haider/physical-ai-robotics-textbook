# Week 5: ROS 2 Fundamentals: Parameters, Launch Files, and CLI Tools

## Learning Outcomes
By the end of this chapter, you will be able to:
1. Understand the concept and usage of ROS 2 parameters.
2. Be able to create and use ROS 2 launch files for multi-node systems.
3. Master essential ROS 2 command-line tools for debugging and introspection.
4. Learn how to remap ROS 2 topics and nodes.

## Introduction: Orchestrating Your Robot's Brain

(FR-006: Contextual Humanity - *Connecting advanced ROS 2 features to real-world robot operation*)
In Weeks 3 and 4, we dove into the communication backbone of ROS 2, mastering nodes, topics, services, and actions. We learned how different parts of a robot's software talk to each other. But how do you configure these software components? How do you start many of them together in a coordinated fashion? And how do you inspect what's happening within your robot's "brain" when things go wrong? This week, we unlock the power of ROS 2 parameters for dynamic configuration, explore launch files for orchestrating complex systems, and master command-line tools for debugging and introspection. These tools are indispensable for developing, deploying, and maintaining sophisticated robotic applications.

## Core Concepts

### 1. ROS 2 Parameters: Dynamic Configuration (FR-005: Modular Mind)

(FR-001: Developmental Staging - *Building from fixed code to dynamic settings*)
Parameters are dynamic, runtime-configurable values that ROS 2 nodes can expose. They allow you to change a node's behavior without recompiling its code. For instance, a navigation node might have a `robot_speed` parameter, which you can adjust on-the-fly to make the robot move faster or slower. Parameters are essentially key-value pairs (`name: value`).

**Key Characteristics**:
*   **Dynamic**: Can be changed during runtime.
*   **Persistent**: Can be loaded from configuration files.
*   **Introspectable**: Can be inspected and modified via CLI tools.

**Analogy**: (FR-004: Creative Synthesis)
Think of a complex household appliance like a washing machine. It has different cycles (programs) that define its behavior. Parameters are like the settings you can adjust for a specific cycle – water temperature, spin speed, etc. – without changing the core functionality of the washing machine.

### 2. ROS 2 Launch Files: Orchestrating Multi-Node Systems (FR-005: Modular Mind)

(FR-001: Developmental Staging - *Moving from single nodes to complex systems*)
As robotic systems grow, you'll have many nodes working together (e.g., camera driver, lidar driver, navigation, manipulation, control). Starting each node manually becomes tedious and error-prone. ROS 2 launch files (written in Python) provide a way to define and start multiple nodes, configure their parameters, and set up communication remappings in a single command.

**Components of a Launch File**:
*   **Nodes**: Specify which executable nodes to start.
*   **Parameters**: Set initial parameter values for nodes.
*   **Remapping**: Change the names of topics, services, or actions used by nodes.
*   **Includes**: Reuse parts of other launch files.
*   **Conditionals**: Start nodes conditionally based on certain criteria.

### 3. Essential ROS 2 Command-Line Tools (FR-005: Modular Mind)

(FR-001: Developmental Staging - *Empowering learners with practical debugging skills*)
ROS 2 provides a powerful suite of command-line interface (CLI) tools, `ros2 <command>`, for interacting with your running system. These are invaluable for development, debugging, and introspection.

**Key CLI Tools**:
*   `ros2 node`: List, info, get/set parameters of nodes.
*   `ros2 topic`: List, echo (view messages), info, pub (publish messages manually).
*   `ros2 service`: List, call, info.
*   `ros2 param`: List, get, set parameters of a node.
*   `ros2 launch`: Start launch files.
*   `ros2 run`: Run a single node.
*   `ros2 pkg`: List, create packages.

## Hands-On Lab: Using ROS 2 Parameters and Launch Files (FR-002: Constructivist Activity)

### Exercise 1: Dynamic Parameters and Basic Launch File (FR-003: Motivational Immersion)

**Challenge Level**: Intermediate

**Objective**: Create a simple ROS 2 node that uses parameters, and then launch it using a Python launch file, dynamically setting its parameters.

**Tools**: ROS 2 development environment. (FR-007: Technology Critique - This exercise directly demonstrates how to make nodes configurable and how to start systems robustly, essential skills for any ROS 2 developer.)

**Steps**:
1.  **Navigate to your `my_first_package`**: `cd ~/ros2_ws/src/my_first_package/my_first_package`
2.  **Create a parameter node (`parameter_node.py`)**:
    ```python
    import rclpy
    from rclpy.node import Node

    class MinimalParameterNode(Node):

        def __init__(self):
            super().__init__('minimal_parameter_node')
            self.declare_parameter('my_parameter', 'world') # Default value is 'world'

            self.timer = self.create_timer(1.0, self.timer_callback)
            self.get_logger().info('Minimal Parameter Node started!')

        def timer_callback(self):
            my_parameter = self.get_parameter('my_parameter').get_parameter_value().string_value
            self.get_logger().info(f'Hello {my_parameter}!')

    def main(args=None):
        rclpy.init(args=args)
        node = MinimalParameterNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```
3.  **Add `parameter_node.py` to `setup.py` entry points**:
    ```python
    # ... (existing setup.py content) ...
    entry_points={
        'console_scripts': [
            'talker = my_first_package.publisher_node:main',
            'listener = my_first_package.subscriber_node:main',
            'parameter_node = my_first_package.parameter_node:main', # New entry
        ],
    },
    # ...
    ```
4.  **Build and source your workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build
    . install/setup.bash
    ```
5.  **Create a `launch` directory in your package**: `mkdir ~/ros2_ws/src/my_first_package/launch`
6.  **Create a launch file (`launch/my_launch_file.py`)**:
    ```python
    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='my_first_package',
                executable='parameter_node',
                name='custom_parameter_node',
                parameters=[
                    {'my_parameter': 'ROS2'} # Override default parameter value
                ],
                output='screen'
            )
        ])
    ```
7.  **Run the launch file**:
    ```bash
    ros2 launch my_first_package my_launch_file.py
    ```

**Expected Output**: The `custom_parameter_node` will start and log "Hello ROS2!" messages, demonstrating that the parameter was successfully overridden by the launch file.

### Exercise 2: Using CLI Tools for Parameters

**Challenge Level**: Beginner

**Objective**: Use ROS 2 command-line tools to inspect and change the parameters of a running node.

**Tools**: The running `parameter_node` from Exercise 1.

**Steps**:
1.  **In a new terminal**, while the launch file from Exercise 1 is running, list all the nodes:
    ```bash
    ros2 node list
    ```
    You should see `/custom_parameter_node` in the list.

2.  **List the parameters** of your node:
    ```bash
    ros2 param list /custom_parameter_node
    ```

3.  **Get the current value** of the `my_parameter` parameter:
    ```bash
    ros2 param get /custom_parameter_node my_parameter
    ```
    This should return `ROS2`.

4.  **Change the parameter's value** at runtime:
    ```bash
    ros2 param set /custom_parameter_node my_parameter "Gemini"
    ```

**Expected Output**: After running the `set` command, the output of your `custom_parameter_node` will change from "Hello ROS2!" to "Hello Gemini!", demonstrating dynamic reconfiguration.

### Exercise 3: Topic Remapping in a Launch File

**Challenge Level**: Intermediate

**Objective**: Use a launch file to remap a topic name for your publisher and subscriber nodes.

**Tools**: The `talker` and `listener` nodes from Week 4.

**Steps**:
1.  **Create a new launch file** named `launch/remapping_launch.py`:
    ```python
    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='my_first_package',
                executable='talker',
                name='my_talker',
                remappings=[
                    ('/topic', '/chatter')
                ]
            ),
            Node(
                package='my_first_package',
                executable='listener',
                name='my_listener',
                remappings=[
                    ('/topic', '/chatter')
                ]
            )
        ])
    ```

2.  **Run the launch file**:
    ```bash
    ros2 launch my_first_package remapping_launch.py
    ```

3.  **In a new terminal**, check the active topics:
    ```bash
    ros2 topic list
    ```
    You should see `/chatter` in the list instead of `/topic`.

**Expected Output**: The `my_listener` node will successfully receive messages from `my_talker` on the `/chatter` topic, demonstrating that the remapping was successful.

## Creative Challenge: Multi-Node Orchestration (FR-004: Creative Synthesis)

**Design Task**: Expand `my_launch_file.py` to start both your `talker` and `listener` nodes from Week 4, along with the `parameter_node`. Use parameter remapping to change the `topic` name for the `talker` and `listener` nodes to `chatter_topic` within the launch file. Verify communication with `ros2 topic echo chatter_topic`.

## Real-World Application: Autonomous Vehicle Software Stack (FR-006: Contextual Humanity)

In an autonomous vehicle, a single launch file can start hundreds of nodes: sensor drivers (camera, lidar, radar), perception algorithms (object detection, lane keeping), planning modules (global and local planners), and control systems. Parameters allow tuning vehicle behavior (e.g., speed limits, following distance), while topic remapping enables flexible integration of different sensor hardware without code changes.

## Technology Deep Dive: ROS 2 Parameter Server vs. Node Parameters (FR-007: Technology Critique)

(FR-001: Developmental Staging - *Clarifying how parameters are managed*)
In ROS 1, there was a global "Parameter Server." In ROS 2, parameters are owned by individual nodes. This decentralized approach offers greater modularity and avoids a single point of failure, improving the robustness of complex systems. Nodes manage their own parameters, which can be dynamically queried and set by other nodes or CLI tools.

## Self-Check Assessment (FR-008: Reflective Assessment)

1.  What is the primary benefit of using ROS 2 parameters, and how do you declare a parameter in a Python node?
2.  Describe the main purpose of a ROS 2 launch file and list at least three types of elements it can define.
3.  Which ROS 2 CLI command would you use to see all the parameters of a specific node?
4.  How would you change the name of a topic from `/camera_images` to `/robot/camera_feed` for a node started within a launch file?

## Before Moving On (FR-008: Reflective Assessment)
- [ ] I can explain what ROS 2 parameters are and how to use them.
- [ ] I have successfully created and launched a ROS 2 node using a launch file.
- [ ] I have used ROS 2 CLI tools for parameter inspection.
- [ ] I understand the concept of topic remapping in launch files.

## Next Steps (FR-001: Developmental Staging)
In the next chapter, we will shift our focus from core ROS 2 concepts to simulating robots in virtual environments, beginning with Gazebo.
