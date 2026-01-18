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
            className="button button--secondary button--lg pulse"
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
        {/* Hero section with animated elements */}
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <div className="text--center padding-horiz--md slide-in-left">
                  <div className={`${styles.featureIcon} float`}>🤖</div>
                  <h3>Embodied Intelligence</h3>
                  <p>Learn how AI systems interact with the physical world through robotics and sensor integration.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className="text--center padding-horiz--md fade-in-up">
                  <div className={`${styles.featureIcon} float`} style={{animationDelay: '0.2s'}}>🌐</div>
                  <h3>ROS 2 Ecosystem</h3>
                  <p>Master the Robot Operating System fundamentals and advanced distributed computing concepts.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className="text--center padding-horiz--md slide-in-right">
                  <div className={`${styles.featureIcon} float`} style={{animationDelay: '0.4s'}}>🧠</div>
                  <h3>Vision-Language-Action</h3>
                  <p>Build systems that understand natural language commands and execute complex robotic tasks.</p>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Comprehensive Learning Path Section */}
        <section className="features-section">
          <div className="container">
            <div className="row">
              <div className="col col--12 text--center padding--vert--xl">
                <h2 className="slide-in-left">Comprehensive Learning Path</h2>
                <p className="padding-horiz--lg fade-in-up">A structured 4-module curriculum taking you from ROS 2 fundamentals to advanced Vision-Language-Action systems for humanoid robotics.</p>
              </div>
            </div>

            <div className="row">
              <div className="col col--3">
                <div className="feature-card slide-in-left" style={{animationDelay: '0.1s'}}>
                  <div className="pulse" style={{display: 'inline-block', fontSize: '2rem', marginBottom: '1rem'}}>🎓</div>
                  <h3>Module 1</h3>
                  <h4>ROS 2 Fundamentals</h4>
                  <p>8-12 hrs/wk • 3 weeks</p>
                  <p>Core ROS 2 architecture, communication patterns, and robot software development</p>
                </div>
              </div>
              <div className="col col--3">
                <div className="feature-card slide-in-left" style={{animationDelay: '0.2s'}}>
                  <div className="pulse" style={{display: 'inline-block', fontSize: '2rem', marginBottom: '1rem'}}>🎮</div>
                  <h3>Module 2</h3>
                  <h4>Simulation & Digital Twins</h4>
                  <p>10-15 hrs/wk • 4 weeks</p>
                  <p>Simulation environments, physics modeling, and sim-to-real transfer</p>
                </div>
              </div>
              <div className="col col--3">
                <div className="feature-card slide-in-left" style={{animationDelay: '0.3s'}}>
                  <div className="pulse" style={{display: 'inline-block', fontSize: '2rem', marginBottom: '1rem'}}>🚀</div>
                  <h3>Module 3</h3>
                  <h4>NVIDIA Isaac Integration</h4>
                  <p>12-15 hrs/wk • 4 weeks</p>
                  <p>GPU-accelerated AI, perception systems, and Isaac platform integration</p>
                </div>
              </div>
              <div className="col col--3">
                <div className="feature-card slide-in-left" style={{animationDelay: '0.4s'}}>
                  <div className="pulse" style={{display: 'inline-block', fontSize: '2rem', marginBottom: '1rem'}}>🤖</div>
                  <h3>Module 4</h3>
                  <h4>Vision-Language-Action Models</h4>
                  <p>15-20 hrs/wk • 8 weeks</p>
                  <p>Multimodal AI, human-robot interaction, and complete embodied systems</p>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Additional animated section for testimonials or achievements */}
        <section className="features-section" style={{background: 'var(--ai-gradient-4)', marginTop: '3rem', padding: '4rem 0'}}>
          <div className="container text--center">
            <h2 className="fade-in-up" style={{color: 'white', marginBottom: '2rem'}}>Why Learn AI Robotics?</h2>
            <div className="row">
              <div className="col col--4">
                <div className="feature-card" style={{background: 'rgba(255, 255, 255, 0.1)', backdropFilter: 'blur(10px)', color: 'white'}}>
                  <div className="float" style={{fontSize: '2.5rem', marginBottom: '1rem'}}>💼</div>
                  <h3>Career Opportunities</h3>
                  <p>High-demand field with excellent job prospects in tech, manufacturing, and research.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className="feature-card" style={{background: 'rgba(255, 255, 255, 0.1)', backdropFilter: 'blur(10px)', color: 'white'}}>
                  <div className="float" style={{fontSize: '2.5rem', marginBottom: '1rem'}}>🔬</div>
                  <h3>Innovation</h3>
                  <p>Be at the forefront of cutting-edge technology that's reshaping industries.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className="feature-card" style={{background: 'rgba(255, 255, 255, 0.1)', backdropFilter: 'blur(10px)', color: 'white'}}>
                  <div className="float" style={{fontSize: '2.5rem', marginBottom: '1rem'}}>🔧</div>
                  <h3>Problem Solving</h3>
                  <p>Develop solutions to real-world challenges that impact society.</p>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}