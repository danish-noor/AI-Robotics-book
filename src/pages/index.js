import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title fade-in-up">{siteConfig.title}</h1>
        <p className="hero__subtitle fade-in-up">{siteConfig.tagline}</p>
        <div className={clsx('margin-horiz--md', styles.buttons, 'fade-in-up')}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning - 4 min ⏱️
          </Link>
          <Link
            className="button button--outline button--secondary button--lg margin-left--sm"
            to="/docs/module-1">
            Explore Modules
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="A Comprehensive Curriculum on AI Robotics">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <div className={styles.featureIcon}>🤖</div>
                  <h3>Embodied Intelligence</h3>
                  <p>Learn how AI systems interact with the physical world through robotics and sensor integration.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <div className={styles.featureIcon}>🌐</div>
                  <h3>ROS 2 Ecosystem</h3>
                  <p>Master the Robot Operating System fundamentals and advanced distributed computing concepts.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <div className={styles.featureIcon}>🧠</div>
                  <h3>Vision-Language-Action</h3>
                  <p>Build systems that understand natural language commands and execute complex robotic tasks.</p>
                </div>
              </div>
            </div>
          </div>
        </section>
        
        <section className="features-section">
          <div className="container">
            <div className="row">
              <div className="col col--12 text--center padding--vert--xl">
                <h2>Comprehensive Learning Path</h2>
                <p className="padding-horiz--lg">A structured 4-module curriculum taking you from ROS 2 fundamentals to advanced Vision-Language-Action systems for humanoid robotics.</p>
              </div>
            </div>
            
            <div className="row">
              <div className="col col--3">
                <div className="feature-card">
                  <h3>Module 1</h3>
                  <h4>ROS 2 Fundamentals</h4>
                  <p>8-12 hrs/wk • 3 weeks</p>
                  <p>Core ROS 2 architecture, communication patterns, and robot software development</p>
                </div>
              </div>
              <div className="col col--3">
                <div className="feature-card">
                  <h3>Module 2</h3>
                  <h4>Simulation & Digital Twins</h4>
                  <p>10-15 hrs/wk • 4 weeks</p>
                  <p>Simulation environments, physics modeling, and sim-to-real transfer</p>
                </div>
              </div>
              <div className="col col--3">
                <div className="feature-card">
                  <h3>Module 3</h3>
                  <h4>NVIDIA Isaac Integration</h4>
                  <p>12-15 hrs/wk • 4 weeks</p>
                  <p>GPU-accelerated AI, perception systems, and Isaac platform integration</p>
                </div>
              </div>
              <div className="col col--3">
                <div className="feature-card">
                  <h3>Module 4</h3>
                  <h4>Vision-Language-Action Models</h4>
                  <p>15-20 hrs/wk • 8 weeks</p>
                  <p>Multimodal AI, human-robot interaction, and complete embodied systems</p>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}