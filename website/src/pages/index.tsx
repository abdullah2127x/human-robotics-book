import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
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
            to="/docs/intro">
            Get Started
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <h3>Module 1: The Robotic Nervous System</h3>
                <p>Learn about ROS and robot operating systems</p>
              </div>
              <div className="col col--4">
                <h3>Module 2: The Digital Twin</h3>
                <p>Understand digital twins in robotics</p>
              </div>
              <div className="col col--4">
                <h3>Module 3: The AI-R</h3>
                <p>Explore AI reasoning in robotics</p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}

