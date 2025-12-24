import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/week-1/physical-ai-foundations">
            Start Learning - Week 1 🚀
          </Link>
        </div>
      </div>
    </header>
  );
}

function HomepageIntroSection(): ReactNode {
  return (
    <section className={clsx('padding-vert--xl', styles.homepageIntro)}>
      <div className="container text--center">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <Heading as="h2" className={styles.homepageIntroTitle}>
              Your Journey into Physical AI & Humanoid Robotics Begins Here
            </Heading>
            <p className="hero__subtitle">
              This AI-native textbook is designed to be a world-class digital learning experience,
              bridging the gap between digital intelligence and physical embodiment. Explore
              pedagogically sound content, interactive exercises, and real-world applications
              to master the exciting field of Physical AI.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="An AI-native digital textbook on Physical AI & Humanoid Robotics, covering ROS 2, Gazebo, NVIDIA Isaac, and advanced humanoid development with interactive exercises and personalized learning.">
      <HomepageHeader />
      <HomepageIntroSection />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
