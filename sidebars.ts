import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro', // General Introduction
    {
      type: 'category',
      label: 'Week 1-2: Physical AI Foundations',
      items: ['week-1/physical-ai-foundations'], // Our newly created chapter
    },
    {
      type: 'category',
      label: 'Weeks 3-5: ROS 2 Fundamentals',
      items: [], // Placeholder for future chapters
    },
    {
      type: 'category',
      label: 'Weeks 6-7: Gazebo & Unity Simulation',
      items: [], // Placeholder for future chapters
    },
    {
      type: 'category',
      label: 'Weeks 8-10: NVIDIA Isaac Platform',
      items: [], // Placeholder for future chapters
    },
    {
      type: 'category',
      label: 'Weeks 11-12: Humanoid Robot Development',
      items: [], // Placeholder for future chapters
    },
    {
      type: 'category',
      label: 'Week 13: Conversational Robotics',
      items: [], // Placeholder for future chapters
    },
  ],
};

export default sidebars;