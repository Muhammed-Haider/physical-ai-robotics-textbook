# Week 2: Physical AI Foundational Concepts

## Learning Outcomes
By the end of this chapter, you will be able to:
1. Explain the role of sensors and actuators in Physical AI systems.
2. Describe different types of robot locomotion and manipulation.
3. Understand the concept of a robot's kinematics and dynamics.
4. Discuss challenges in perception and control for physical robots.

## Introduction: The Building Blocks of Embodiment

:::info FR-001: Developmental Staging
In Week 1, we established the foundational understanding of Physical AI and Humanoid Robotics, differentiating it from purely digital AI. We explored the 'why' behind bringing AI into the physical world. This week, we delve into the 'how'. What are the fundamental components that allow an AI to perceive its environment, move, and interact? We will dissect the essential hardware and software elements, starting with the senses and actions of a physical system: sensors and actuators.
:::

## Core Concepts

### 1. Sensing the World: The Role of Sensors :::info FR-005: Modular Mind
:::

:::info FR-001: Developmental Staging
Sensors are the "eyes, ears, and touch" of a Physical AI system. They convert physical phenomena (light, sound, pressure, temperature, distance) into electrical signals that an AI can process. Without accurate and reliable sensory input, a robot would be blind, deaf, and oblivious to its surroundings.
:::

**Types of Sensors**:
*   **Proprioceptive Sensors**: (Internal state)
    *   **Encoders**: Measure joint angles and motor rotation (how its own body is moving).
    *   **IMUs (Inertial Measurement Units)**: Combine accelerometers and gyroscopes to measure orientation, angular velocity, and linear acceleration (how its body is oriented and accelerating in space).
    *   **Force/Torque Sensors**: Measure forces applied at joints or end-effectors (how much pressure its hand is exerting).
*   **Exteroceptive Sensors**: (External environment)
    *   **Cameras**: Provide visual information (what's around it). Types include monocular, stereo, and depth cameras (RGB-D).
    *   **Lidar (Light Detection and Ranging)**: Uses pulsed laser to measure distances to objects, creating 3D maps of the environment.
    *   **Ultrasonic Sensors**: Use sound waves to measure distance, common for obstacle detection.
    *   **Tactile Sensors**: Detect touch and pressure, crucial for grasping and manipulation.

:::info FR-004: Creative Synthesis
**Analogy**: Imagine navigating a dark room. Your eyes are useless (like a camera in low light), but you use your hands (tactile sensors) to feel for walls and objects, and your internal sense of balance (IMU) to stay upright. This multi-sensory approach is vital for Physical AI.
:::

### 2. Acting on the World: The Role of Actuators :::info FR-005: Modular Mind
:::

Actuators are the "muscles" of a Physical AI system. They convert electrical signals from the AI into physical motion or force. Common types include electric motors, hydraulic cylinders, and pneumatic pistons. The choice of actuator depends on the required power, precision, and speed.

**Types of Actuators**:
*   **Electric Motors**:
    *   **DC Motors**: Simple, common.
    *   **Servo Motors**: Provide precise angular positioning.
    *   **Stepper Motors**: Provide precise incremental motion.
*   **Hydraulic Actuators**: Use incompressible fluid (oil) for high force applications.
*   **Pneumatic Actuators**: Use compressible fluid (air) for high speed and light loads.

### 3. Locomotion and Manipulation :::info FR-002: Constructivist Activity
:::

These are two primary ways robots interact physically.
*   **Locomotion**: How a robot moves itself through its environment (e.g., wheels, legs, tracks, flying).
*   **Manipulation**: How a robot moves objects within its environment (e.g., grippers, robotic arms).

**Exercise 1: Sensor Data Interpretation :::info FR-003: Motivational Immersion
:::**

**Challenge Level**: Beginner

**Objective**: Simulate basic sensor readings and interpret them to make a simple decision.

**Tools**: Python. :::info FR-007: Technology Critique - Python is chosen for its simplicity and wide use in robotics scripting, allowing direct focus on sensor interpretation logic.
:::

**Steps**:
1.  **Install the library**:
    ```bash
    pip install pybullet
    ```
2.  **Create a Python script (`sensor_decision.py`)**:
    ```python
    import random
    import time

    def read_ultrasonic_sensor():
        # Simulate reading distance in cm (e.g., from 10 to 200 cm)
        return random.randint(10, 200)

    def main():
        print("Starting obstacle detection simulation...")
        for _ in range(10): # Simulate 10 readings
            distance = read_ultrasonic_sensor()
            print(f"Distance detected: {distance} cm")

            if distance < 30:
                print("OBSTACLE DETECTED! Stopping or re-routing.")
                # In a real robot, this would trigger an action
            elif distance < 70:
                print("Obstacle near. Proceeding with caution.")
            else:
                print("Path clear. Moving forward.")
            time.sleep(1) # Wait for 1 second before next reading
        print("Simulation ended.")

    if __name__ == "__main__":
        main()
    ```
3.  **Run the script**:
    ```bash
    python sensor_decision.py
    ```

**Expected Output**: The script will print simulated distance readings and a decision based on those readings (e.g., "OBSTACLE DETECTED!", "Proceeding with caution.", "Path clear.").

### 4. Kinematics and Dynamics :::info FR-005: Modular Mind
:::

:::info FR-001: Developmental Staging
To control a robot, we need to understand its motion.
*   **Kinematics**: Describes the motion of a robot without considering the forces that cause that motion.
    *   **Forward Kinematics**: Given the joint angles, what is the position and orientation of the end-effector (e.g., hand)?
    *   **Inverse Kinematics**: Given the desired position and orientation of the end-effector, what are the required joint angles? This is often more complex.
*   **Dynamics**: Deals with the forces and torques that cause motion. It considers mass, inertia, and external forces.
:::

**Visual Aid**: (FR-005: Modular Mind - *This is a placeholder for a diagram or visual explanation of Forward vs. Inverse Kinematics*)

## Creative Challenge: Simple Actuator Control :::info FR-004: Creative Synthesis
:::

**Design Task**: Modify `sensor_decision.py`. Imagine your robot has two "motors" (actuators) that can move it forward (by setting speed > 0) or turn (by setting different speeds for left/right motors). Can you make your simulated robot avoid an obstacle by turning, rather than just stopping?

### Exercise 2: Reactive Actuation Based on Sensor Data :::info FR-003: Motivational Immersion
:::

**Challenge Level**: Intermediate

**Objective**: Modify the `sensor_decision.py` script to make the simulated robot react to detected obstacles by turning, demonstrating a basic sense-act loop.

**Tools**: Python. :::info FR-007: Technology Critique - Python's readability makes it ideal for illustrating sensor-actuator logic without complex overhead.
:::

**Steps**:
1.  **Copy `sensor_decision.py`**: Duplicate `sensor_decision.py` and name the new file `exercise2_reactive_actuation.py`.
2.  **Modify `main()` function**: Open `exercise2_reactive_actuation.py` and replace the `main()` function with the following. This version will simulate movement (linear and angular velocity) based on sensor input.
    ```python
    def main():
        print("Starting reactive obstacle avoidance simulation...")
        linear_velocity = 0.1 # m/s
        angular_velocity = 0.0 # rad/s

        for _ in range(20): # Simulate 20 readings/time steps
            distance = read_ultrasonic_sensor()
            print(f"Distance detected: {distance} cm")

            if distance < 30: # Obstacle very close, turn sharply
                print("OBSTACLE DETECTED! Turning right.")
                linear_velocity = 0.0
                angular_velocity = -0.5 # Turn right
            elif distance < 70: # Obstacle near, turn gently
                print("Obstacle near. Turning slightly left.")
                linear_velocity = 0.05
                angular_velocity = 0.2 # Turn left
            else: # Path clear, move forward
                print("Path clear. Moving forward.")
                linear_velocity = 0.1
                angular_velocity = 0.0
            
            # Simulate robot movement based on calculated velocities (conceptual)
            print(f"Robot command: Linear={linear_velocity:.2f} m/s, Angular={angular_velocity:.2f} rad/s")
            time.sleep(0.5) # Wait for 0.5 second before next reading
        print("Reactive simulation ended.")

    if __name__ == "__main__":
        main()
    ```
3.  **Run the script**:
    ```bash
    python exercise2_reactive_actuation.py
    ```

**Expected Output**: The script will print simulated distance readings. For each reading, it will also print a decision (e.g., "OBSTACLE DETECTED! Turning right.") and conceptual robot movement commands (linear and angular velocities) showing how the robot would react.

### Exercise 3: Simple Sensor Fusion / State Inference :::info FR-003: Motivational Immersion
:::

**Challenge Level**: Intermediate

**Objective**: Combine readings from two simulated "ultrasonic" sensors to infer a more robust obstacle presence and direction.

**Tools**: Python. :::info FR-007: Technology Critique - This exercise highlights how combining simple sensors can lead to richer environmental understanding, a foundation for sensor fusion.
:::

**Steps**:
1.  **Create a new script**: Create a new Python file named `exercise3_sensor_fusion.py`.
2.  **Define sensor functions**: Add the `read_ultrasonic_sensor()` function from `sensor_decision.py`. Then, add a second function `read_left_ultrasonic_sensor()` which also returns a random distance, but simulates a sensor on the left side.
    ```python
    import random
    import time

    def read_front_ultrasonic_sensor():
        return random.randint(20, 150) # Simulate front sensor

    def read_left_ultrasonic_sensor():
        return random.randint(20, 200) # Simulate left sensor

    def main():
        print("Starting simple sensor fusion simulation...")
        for _ in range(10):
            front_distance = read_front_ultrasonic_sensor()
            left_distance = read_left_ultrasonic_sensor()
            print(f"Front Distance: {front_distance} cm, Left Distance: {left_distance} cm")

            if front_distance < 40 and left_distance < 40:
                print("Obstacle directly ahead and to the left! Critical avoidance needed.")
            elif front_distance < 50:
                print("Obstacle directly ahead. Proceed with caution.")
            elif left_distance < 60:
                print("Obstacle on the left side. Adjust path to the right.")
            else:
                print("Path appears clear.")
            time.sleep(1)
        print("Sensor fusion simulation ended.")

    if __name__ == "__main__":
        main()
    ```
3.  **Run the script**:
    ```bash
    python exercise3_sensor_fusion.py
    ```

**Expected Output**: The script will print simulated front and left sensor readings. Based on these combined readings, it will infer and print a more detailed state about obstacles, demonstrating how multiple sensor inputs lead to better decision-making.

## Real-World Application: Prosthetics and Exoskeletons :::info FR-006: Contextual Humanity
:::

Advanced prosthetics and exoskeletons are prime examples of Physical AI. They rely on sophisticated sensors (e.g., EMG signals from muscles, pressure sensors) and actuators (miniature motors, hydraulics) to provide natural, intuitive motion and feedback to the user. Understanding kinematics and dynamics is crucial for making these devices move naturally and safely.

## Technology Deep Dive: Data Filtering and Fusion :::info FR-007: Technology Critique
:::

Real-world sensor data is noisy and imperfect. Physical AI systems often employ techniques like Kalman Filters or Extended Kalman Filters to combine data from multiple sensors (e.g., IMU and GPS) to get a more accurate estimate of the robot's state (position, velocity, orientation). This is a critical step for reliable autonomous operation.

## Self-Check Assessment :::info FR-008: Reflective Assessment
:::

1.  What is the primary difference between a cloud-based AI service and a Physical AI system?
2.  Name one historical event or invention that significantly contributed to the rise of Physical AI.
3.  Why is simulation considered a critical tool in Physical AI development?

## Before Moving On :::info FR-008: Reflective Assessment
:::
- [ ] I can list and describe common types of sensors and actuators.
- [ ] I understand the basic principles of locomotion and manipulation.
- [ ] I have successfully run the sensor simulation and interpreted its output.
- [ ] I understand the basic concepts of kinematics and dynamics.

## Next Steps :::info FR-001: Developmental Staging
:::
In the upcoming chapters, we will apply these foundational concepts using practical tools like the Robot Operating System (ROS 2) and advanced simulation environments.