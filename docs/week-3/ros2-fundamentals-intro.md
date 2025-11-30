# Week 3: ROS 2 Fundamentals: Introduction and Core Concepts

## Learning Outcomes
By the end of this chapter, you will be able to:
1. Understand the fundamental architecture and components of ROS 2.
2. Explain the concepts of nodes, topics, and messages in ROS 2.
3. Be able to set up a basic ROS 2 workspace and package.
4. Learn how to publish and subscribe to ROS 2 topics.
5. Understand how to interact with ROS 2 through command-line tools.

## Introduction: The Operating System for Robots

(FR-006: Contextual Humanity - *Connecting ROS 2 to the broader robotics ecosystem*)
In Weeks 1 and 2, we explored the 'what' and 'how' of Physical AI, from its core definitions to the role of sensors and actuators. Now, as we begin to build more complex robotic systems, we encounter the challenge of managing multiple hardware components, software processes, and data streams simultaneously. This is where the Robot Operating System (ROS) comes in. ROS 2 is not a traditional operating system like Windows or Linux, but rather a flexible framework for writing robot software. It provides tools, libraries, and conventions that simplify the task of creating complex and robust robot applications. This chapter will introduce you to the fundamental architecture and core concepts of ROS 2, preparing you to build your first robotic applications.

## Core Concepts

### 1. What is ROS 2? (FR-005: Modular Mind)

(FR-001: Developmental Staging - *Starting with the basics before diving into components*)
ROS 2 (Robot Operating System 2) is an open-source set of software libraries and tools that help you build robot applications. It aims to standardize how robotic components communicate, making it easier to develop, test, and deploy complex robotics projects. Think of it as a middleware that sits between your robot's hardware and your application code, handling communication, device drivers, libraries, and visualization tools.

**Analogy**: (FR-004: Creative Synthesis)
Imagine building a house. Instead of building every brick, wire, and pipe from scratch, you use standardized components (pipes, wires, timber) and tools (saws, drills). ROS 2 provides the "standardized components" and "tools" for robotics, so you don't have to reinvent the wheel for every new robot.

### 2. ROS 2 Architecture: The Communication Graph (FR-005: Modular Mind)

The core of ROS 2 is its communication architecture, often visualized as a "computation graph." This graph consists of several key elements:

*   **Nodes**: (FR-001: Developmental Staging - *Explaining the smallest unit first*)
    Nodes are individual executable processes that perform a specific task. For example, one node might control a motor, another might process camera data, and a third might handle navigation. Each node is independent and can run on different machines.

*   **Topics**:
    Topics are named buses over which nodes exchange messages. Messages are data structures (like a simple Python object) that contain information (e.g., sensor readings, motor commands). Nodes communicate anonymously via topics: a node publishes messages to a topic, and other nodes subscribe to that topic to receive the messages. Publishers and subscribers don't know about each other directly.

*   **Messages**:
    Messages are the data types used for communication over topics. ROS 2 provides a rich set of standard message types for common robotics data (e.g., `sensor_msgs/msg/LaserScan`, `geometry_msgs/msg/Twist`). You can also define custom message types.

*   **Services**:
    Services provide a request/response communication model, similar to a client-server interaction. A client sends a request to a service, and the service performs an action and sends back a response. This is used for operations that require a direct reply, like asking a robot to perform a specific action once.

*   **Actions**:
    Actions are similar to services but are designed for long-running tasks. An action client sends a goal to an action server, which provides continuous feedback on the progress of the goal and a final result when completed. This is useful for tasks like "navigate to a specific location," where you want updates along the way.

**Visual Aid**: (FR-005: Modular Mind - *Placeholder for a diagram illustrating ROS 2 communication graph with nodes, topics, and messages*)
<!-- Diagram: A conceptual diagram illustrating multiple nodes (boxes) connected by topics (arrows) representing data flow. Show publishers and subscribers. -->

### 3. Setting Up Your ROS 2 Workspace (FR-002: Constructivist Activity)

To work with ROS 2, you'll typically set up a "workspace" – a directory where you develop your custom packages.

**Exercise 1: Initialize a ROS 2 Workspace and Package (FR-003: Motivational Immersion)**

**Challenge Level**: Beginner

**Objective**: Create a ROS 2 workspace and your first custom package in it.

**Tools**: A Linux environment (Ubuntu 22.04 LTS is recommended) with ROS 2 Humble installed. (FR-007: Technology Critique - Linux and ROS 2 are the standard development environment for robotics. This exercise is foundational for all subsequent ROS 2 work.)

**Steps**:
1.  **Open a terminal** and create a new directory for your workspace:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```
    The `-p` flag ensures that both `ros2_ws` and `ros2_ws/src` are created. The `src` directory is where your ROS 2 packages will reside.

2.  **Create a new ROS 2 package**:
    ```bash
    ros2 pkg create --build-type ament_python my_first_package
    ```
    This command creates a new Python package named `my_first_package`. You can inspect the `my_first_package` directory to see its basic structure (e.g., `setup.py`, `package.xml`).

3.  **Build the workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build
    ```
    `colcon` is the build tool used in ROS 2. `colcon build` compiles all packages in your workspace.

4.  **Source the workspace**:
    ```bash
    . install/setup.bash
    ```
    This command adds your workspace's environment variables to your current shell session, allowing you to use your newly built packages. You typically add this to your `~/.bashrc` file for convenience.

**Expected Output**: The `colcon build` command should complete successfully without errors. After sourcing, you should be able to run `ros2 pkg list | grep my_first_package` and see your package listed.

## Creative Challenge: Exploring ROS 2 Commands (FR-004: Creative Synthesis)

**Design Task**: Without looking up the answers, use ROS 2 command-line tools (e.g., `ros2 help`, `ros2 topic list`, `ros2 node list`) to discover what other default nodes and topics might be running on a fresh ROS 2 installation. How would you determine the data type of a message published on a topic?

## Real-World Application: Autonomous Navigation (FR-006: Contextual Humanity)

Consider an autonomous mobile robot navigating a factory floor. Its ability to achieve its goals relies heavily on ROS 2. A "localization node" publishes the robot's position on a `/tf` topic, while a "sensor fusion node" subscribes to `/lidar_scans` and `/imu_data` topics, and a "path planning node" publishes `geometry_msgs/msg/Twist` messages to a `/cmd_vel` topic to control the robot's movement. All these components communicate seamlessly through the ROS 2 framework.

## Technology Deep Dive: DDS - The Backbone of ROS 2 (FR-007: Technology Critique)

(FR-001: Developmental Staging - *Explain the underlying technology*)
Unlike ROS 1, which had its own communication system, ROS 2 leverages an industry standard called DDS (Data Distribution Service). DDS provides features like quality of service (QoS) policies, security, and real-time communication, which are crucial for robust robotic applications. This choice allows ROS 2 to be more flexible, scalable, and reliable, especially in multi-robot systems and environments with strict timing requirements.

## Self-Check Assessment (FR-008: Reflective Assessment)

1.  What is the primary purpose of a ROS 2 node, and how do nodes communicate with each other?
2.  Describe the difference between ROS 2 topics, services, and actions. When would you use each?
3.  You've created a new ROS 2 package. What command would you use to compile your workspace and make your package available?

## Before Moving On (FR-008: Reflective Assessment)
- [ ] I can describe the core components of ROS 2 architecture (nodes, topics, messages).
- [ ] I have successfully created a ROS 2 workspace and my first custom package.
- [ ] I understand the basic `colcon build` and `source` commands.
- [ ] I have explored ROS 2 command-line tools.

## Next Steps (FR-001: Developmental Staging)
In the upcoming chapters, we will dive deeper into ROS 2 communication, learning how to create nodes that publish and subscribe to topics, and how to define custom message types.
