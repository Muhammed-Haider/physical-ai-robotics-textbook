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
      items: ['week-3/ros2-fundamentals-intro', 'week-4/ros2-communication-patterns'], // Our newly created chapter
    },
    {
      type: 'category',
      label: 'Weeks 6-7: Gazebo & Unity Simulation',
      items: [], // Placeholder for future chapters
      link: {
        type: 'generated-index',
        title: 'Gazebo & Unity Simulation Overview',
      },
    },
    {
      type: 'category',
      label: 'Weeks 8-10: NVIDIA Isaac Platform',
      items: [], // Placeholder for future chapters
      link: {
        type: 'generated-index',
        title: 'NVIDIA Isaac Platform Overview',
      },
    },
    {
      type: 'category',
      label: 'Weeks 11-12: Humanoid Robot Development',
      items: [], // Placeholder for future chapters
      link: {
        type: 'generated-index',
        title: 'Humanoid Robot Development Overview',
      },
    },
    {
      type: 'category',
      label: 'Week 13: Conversational Robotics',
      items: [], // Placeholder for future chapters
      link: {
        type: 'generated-index',
        title: 'Conversational Robotics Overview',
      },
    },
  ],
};

export default sidebars;