import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Easy to Learn',
    Svg: require('../../static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Our curriculum is designed to take you from beginner to advanced levels
        in AI robotics with hands-on exercises and practical examples.
      </>
    ),
  },
  {
    title: 'Focus on Best Practices',
    Svg: require('../../static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Learn industry-standard practices for developing robust and scalable
        robotic systems using modern tools and frameworks.
      </>
    ),
  },
  {
    title: 'Powered by Open Source',
    Svg: require('../../static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Built with the latest open-source technologies including ROS 2, NVIDIA Isaac,
        and state-of-the-art AI frameworks.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} alt={title} />
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}