# Week 1: Physical AI Foundations

This chapter introduces the fundamental concepts of Physical AI.

## From Bits to Atoms (FR-001, FR-006)

We begin with a simple idea: what happens when AI, which traditionally lives in the world of bits and data, starts to interact with the physical world of atoms? This is the core of Physical AI. We'll start with simple examples and build up to more complex ideas.

## Your First "Physical" Program (FR-002)

Let's write a program that controls a virtual robot arm. This isn't just theory; you'll be writing code that has a direct, visible effect in a simulated environment.

```python
# A simple program to move a virtual robot arm
def move_arm(x, y, z):
  """Moves the robot arm to the specified coordinates."""
  print(f"Moving arm to ({x}, {y}, {z})")
  # In a real simulation, this would call the robot's API
  return True

move_arm(0.5, 0.2, 1.0)
```

## The "Flow State" Challenge (FR-003)

The exercise above gives you immediate feedback (the print statement). Now, try to modify it to move the arm to three different points in sequence. It's a small step up in difficulty to keep you engaged.

## The Robot as a Painter (FR-004)

Imagine the robot arm is holding a paintbrush. Can you write a program to make it draw a simple shape, like a square? This open-ended problem encourages you to think creatively about how to use the tools you've learned.

## Mental Model: The Robot's World (FR-005)

Think of the robot's world as a 3D grid. The `move_arm` function is your way of telling the robot where to go in that grid.

![Robot Arm Coordinate System](https://i.imgur.com/k6gXy5c.png)

This diagram helps visualize the robot's coordinate system.

## Why Does This Matter? (FR-006)

From self-driving cars to automated warehouses, Physical AI is transforming industries. Understanding these principles is the first step to being a part of that transformation.

## Why This Simulation? (FR-007)

We are using a simple, text-based simulation for now to focus purely on the logic of controlling a physical system, without the overhead of a complex 3D environment. This lets us learn the core concepts faster.

## Check Your Understanding (FR-008)

- What is the main difference between traditional AI and Physical AI?
- What does it mean to "scaffold" learning?
- How can immediate feedback help you learn?
