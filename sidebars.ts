import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro', // General Introduction
    {
      type: 'category',
      label: 'Week 1: Physical AI Foundations',
      items: ['week-1/physical-ai-foundations'],
    },
    {
      type: 'category',
      label: 'Week 2: Physical AI Foundations Cont.',
      items: ['week-2/physical-ai-foundational-concepts'],
    },
    {
      type: 'category',
      label: 'Week 3: ROS 2 Fundamentals',
      items: ['week-3/ros2-fundamentals-intro'],
    },
    {
      type: 'category',
      label: 'Week 4: ROS 2 Communication Patterns',
      items: ['week-4/ros2-communication-patterns'],
    },
    {
      type: 'category',
      label: 'Week 5: ROS 2 Parameters, Launch, CLI',
      items: ['week-5/ros2-parameters-launch-cli'],
    },
    {
      type: 'category',
      label: 'Week 6: Gazebo Introduction & Setup',
      items: ['week-6/gazebo-intro-setup'],
    },
    {
      type: 'category',
      label: 'Week 7: Gazebo & Unity Advanced',
      items: ['week-7/gazebo-unity-advanced'],
    },
    {
      type: 'category',
      label: 'Week 8: NVIDIA Isaac Platform Intro',
      items: ['week-8/isaac-platform-intro'],
    },
    {
      type: 'category',
      label: 'Week 9: NVIDIA Isaac ROS 2 Advanced',
      items: ['week-9/isaac-ros2-advanced'],
    },
    {
      type: 'category',
      label: 'Week 10: NVIDIA Isaac Multi-Robot & AI Training',
      items: ['week-10/isaac-multi-robot-ai-training'],
    },
    {
      type: 'category',
      label: 'Week 11: Humanoid Design & Actuation',
      items: ['week-11/humanoid-design-actuation'],
    },
    {
      type: 'category',
      label: 'Week 12: Humanoid Perception, Control & Interaction',
      items: ['week-12/humanoid-perception-control-interaction'],
    },
    {
      type: 'category',
      label: 'Week 13: Conversational Robotics & NLP',
      items: ['week-13/conversational-robotics-nlp'],
    },
  ],
};

export default sidebars;