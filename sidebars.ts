import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro', // General Introduction
    {
      type: 'category',
      label: 'Week 1-2: Physical AI Foundations',
      items: ['week-1/physical-ai-foundations', 'week-2/physical-ai-foundational-concepts'], // Our newly created chapter
    },
    {
      type: 'category',
      label: 'Weeks 3-5: ROS 2 Fundamentals',
      items: ['week-3/ros2-fundamentals-intro', 'week-4/ros2-communication-patterns', 'week-5/ros2-parameters-launch-cli'], // Our newly created chapter
    },
    {
      type: 'category',
      label: 'Weeks 6-7: Gazebo & Unity Simulation',
      items: ['week-6/gazebo-intro-setup', 'week-7/gazebo-unity-advanced'], // Our newly created chapter
    },
    {
      type: 'category',
      label: 'Weeks 8-10: NVIDIA Isaac Platform',
      items: ['week-8/isaac-platform-intro', 'week-9/isaac-ros2-advanced', 'week-10/isaac-multi-robot-ai-training'], // Our newly created chapter
    },
    {
      type: 'category',
      label: 'Weeks 11-12: Humanoid Robot Development',
      items: ['week-11/humanoid-design-actuation', 'week-12/humanoid-perception-control-interaction'], // Our newly created chapter
    },
    {
      type: 'category',
      label: 'Week 13: Conversational Robotics',
      items: ['week-13/conversational-robotics-nlp'], // Our newly created chapter
    },
  ],
};

export default sidebars;