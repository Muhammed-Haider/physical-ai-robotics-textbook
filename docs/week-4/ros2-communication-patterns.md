# Week 4: ROS 2 Fundamentals: Communication Patterns (Topics, Services, Actions)

## Learning Outcomes
By the end of this chapter, you will be able to:
1. Implement ROS 2 nodes that publish and subscribe to topics.
2. Create custom ROS 2 message types.
3. Implement ROS 2 services for request/response communication.
4. Understand the use cases for ROS 2 actions in long-running tasks.

## Introduction: The Language of ROS 2 Robots

(FR-006: Contextual Humanity - *Connecting ROS 2 communication to how robots coordinate*)
In Week 3, we laid the foundation for ROS 2, understanding its architecture and setting up our first workspace. We learned that ROS 2 is essentially a sophisticated communication framework that allows different parts of a robot's software to talk to each other. This week, we will dive deep into the diverse ways ROS 2 facilitates this communication: through Topics, Services, and Actions. Mastering these communication patterns is crucial for developing robust, distributed, and scalable robotic applications, enabling robots to share sensor data, request tasks, and execute complex behaviors collaboratively.

## Core Concepts

### 1. Topics: Asynchronous Data Streaming (FR-005: Modular Mind)

(FR-001: Developmental Staging - *Building on the basic definition from Week 3*)
Topics are the most common communication mechanism in ROS 2. They provide a publish-subscribe model, meaning nodes can send (publish) messages to a named topic, and other nodes can receive (subscribe to) messages from that topic. This is an asynchronous, one-to-many communication pattern, ideal for continuous streams of data like sensor readings (e.g., camera images, laser scans) or periodic status updates.

**Key Characteristics**:
*   **Asynchronous**: Publishers don't wait for subscribers.
*   **One-to-many**: One publisher can have multiple subscribers.
*   **Decoupled**: Publishers and subscribers don't need direct knowledge of each other.

**Analogy**: (FR-004: Creative Synthesis)
Think of a public radio broadcast. The radio station (publisher) broadcasts continuously on a specific frequency (topic). Many people (subscribers) can tune into that frequency to listen, without the station needing to know who is listening or waiting for anyone.

### 2. Creating Custom Message Types (FR-002: Constructivist Activity)

While ROS 2 provides many standard message types, you often need to define custom messages to exchange application-specific data.

**Exercise 1: Define a Custom Message Type (FR-003: Motivational Immersion)**

**Challenge Level**: Beginner

**Objective**: Create a custom ROS 2 message definition for a simple robot status.

**Tools**: ROS 2 development environment (Ubuntu with ROS 2 Humble). (FR-007: Technology Critique - Defining custom messages is fundamental for specialized robotic applications, allowing clear and efficient data representation.)

**Steps**:
1.  **Navigate to your ROS 2 package**:
    ```bash
    cd ~/ros2_ws/src/my_first_package
    mkdir msg
    ```
2.  **Create a message definition file (`msg/RobotStatus.msg`)**:
    ```
    # RobotStatus.msg
    string robot_name
    uint8 battery_level # 0-100
    bool is_charging
    string status_message
    ```
3.  **Edit `package.xml`**: Add these lines inside the `<build_depends>` and `<exec_depends>` sections, and also add a `<build_type>`:
    ```xml
    <build_depend>rosidl_default_generators</build_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```
    Add or modify `<build_type>` to `<build_type>ament_cmake</build_type>` if your package is C++, or keep `ament_python` if you're mixing. For message generation, `ament_cmake` is often used, but it can work with `ament_python` too by ensuring `rosidl_default_generators` is in `build_depend`.

4.  **Edit `CMakeLists.txt`**: Add the following lines to enable message generation:
    ```cmake
    find_package(rosidl_default_generators REQUIRED)
    rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/RobotStatus.msg"
    )
    ```

5.  **Build your workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build
    ```
    This step will generate the necessary C++ and Python headers/code for your custom message.

**Expected Output**: The `colcon build` should succeed. You can verify the message generation by trying to `ros2 interface show my_first_package/msg/RobotStatus`.

### 3. Services: Request-Response Interactions (FR-005: Modular Mind)

Services provide a synchronous, one-to-one request/response communication. A node acts as a "service server" offering a specific function, and another node acts as a "service client" requesting that function. This is suitable for operations that need an immediate result, like "turn on a light" and getting a "light is on" confirmation.

**Analogy**: (FR-004: Creative Synthesis)
Think of ordering food at a restaurant. You (client) make a request to the waiter (service server). The waiter takes your order, the kitchen prepares it, and the waiter brings you back your food (response). This is a direct, one-time interaction.

### 4. Actions: Long-Running Goal-Oriented Tasks (FR-005: Modular Mind)

Actions are built on top of services and topics, designed for long-running, goal-oriented tasks that provide periodic feedback and can be preempted. For example, "drive to the kitchen" is an action. The robot provides feedback ("I'm 20% there", "I'm turning left") and can be cancelled before completion.

**Structure of an Action**:
*   **Goal**: What the client wants the server to achieve.
*   **Result**: What the server sends back when the goal is completed.
*   **Feedback**: Continuous updates on the progress of the goal.

## Hands-On Lab: Implementing ROS 2 Publishers and Subscribers (FR-002: Constructivist Activity)

### Exercise 2: Basic Publisher and Subscriber (FR-003: Motivational Immersion)

**Challenge Level**: Intermediate

**Objective**: Write a ROS 2 Python node that publishes "Hello, World!" messages and another node that subscribes to and prints them.

**Tools**: ROS 2 development environment. (FR-007: Technology Critique - This exercise directly teaches the fundamental publish-subscribe pattern, essential for distributed robot control.)

**Steps**:
1.  **Navigate to your package**: `cd ~/ros2_ws/src/my_first_package/my_first_package`
2.  **Create a publisher node (`publisher_node.py`)**:
    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String # Standard ROS 2 string message type

    class MinimalPublisher(Node):

        def __init__(self):
            super().__init__('minimal_publisher')
            self.publisher_ = self.create_publisher(String, 'topic', 10) # topic name, QoS history depth
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0

        def timer_callback(self):
            msg = String()
            msg.data = 'Hello World: %d' % self.i
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1

    def main(args=None):
        rclpy.init(args=args)
        minimal_publisher = MinimalPublisher()
        rclpy.spin(minimal_publisher) # Keeps node alive until Ctrl+C
        minimal_publisher.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```
3.  **Create a subscriber node (`subscriber_node.py`)**:
    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class MinimalSubscriber(Node):

        def __init__(self):
            super().__init__('minimal_subscriber')
            self.subscription = self.create_subscription(
                String,
                'topic', # topic name
                self.listener_callback,
                10) # QoS history depth
            self.subscription  # prevent unused variable warning

        def listener_callback(self, msg):
            self.get_logger().info('I heard: "%s"' % msg.data)

    def main(args=None):
        rclpy.init(args=args)
        minimal_subscriber = MinimalSubscriber()
        rclpy.spin(minimal_subscriber)
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```
4.  **Edit `setup.py`**: Add entry points for your nodes.
    ```python
    # ... (existing setup.py content) ...
    entry_points={
        'console_scripts': [
            'talker = my_first_package.publisher_node:main',
            'listener = my_first_package.subscriber_node:main',
        ],
    },
    # ...
    ```
5.  **Build and source your workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build
    . install/setup.bash
    ```
6.  **Run the nodes (in separate terminals)**:
    ```bash
    ros2 run my_first_package talker
    ros2 run my_first_package listener
    ```

**Expected Output**: The `talker` terminal will show "Publishing: Hello World: X" messages, and the `listener` terminal will show "I heard: Hello World: X" messages, demonstrating successful communication.

## Creative Challenge: Custom Message Communication (FR-004: Creative Synthesis)

**Design Task**: Modify Exercise 2 to use your `RobotStatus.msg` custom message. Create a publisher that sends `RobotStatus` messages and a subscriber that receives and interprets them. Can you add logic to the subscriber to react differently based on the `battery_level` or `is_charging` status?

## Real-World Application: Multi-Robot Coordination (FR-006: Contextual Humanity)

In scenarios like drone swarms or collaborative robot arms in a factory, ROS 2 communication patterns are indispensable. Topics allow drones to share their positions, services enable a lead drone to assign tasks to others, and actions manage long-duration missions with continuous feedback, ensuring the entire swarm operates as a cohesive unit.

## Technology Deep Dive: Quality of Service (QoS) in ROS 2 (FR-007: Technology Critique)

(FR-001: Developmental Staging - *Delving into advanced communication reliability*)
ROS 2's use of DDS brings powerful Quality of Service (QoS) settings. QoS policies control aspects of communication like reliability (guaranteed delivery vs. best effort), history (keeping all messages vs. only the last N), and durability (sending past messages to new subscribers). Understanding and configuring QoS is vital for designing robust and predictable real-time robotic systems, especially in mission-critical applications.

## Self-Check Assessment (FR-008: Reflective Assessment)

1.  Describe a scenario where you would prefer using a ROS 2 Service over a Topic.
2.  What are the three main components of a ROS 2 Action, and why are Actions suitable for long-running tasks?
3.  You've created a custom `.msg` file. What are the key steps in `package.xml` and `CMakeLists.txt` to enable ROS 2 to recognize and generate code for it?

## Before Moving On (FR-008: Reflective Assessment)
- [ ] I can describe the difference between ROS 2 Topics, Services, and Actions.
- [ ] I have successfully defined a custom ROS 2 message type.
- [ ] I have implemented a basic ROS 2 publisher and subscriber using Python.
- [ ] I understand how to interpret and respond to simple sensor data in a ROS 2 context.

## Next Steps (FR-001: Developmental Staging)
In the next chapter, we will continue our exploration of ROS 2, focusing on advanced concepts like parameters, launch files, and basic client libraries for interacting with the ROS 2 ecosystem more effectively.
