import type { ReactNode } from 'react';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';

import styles from './index.module.css';

function HeroSection() {
  return (
    <section className={styles.hero}>
      <div className={styles.heroContent}>
        <p className={styles.challenge}>The Challenge</p>
        <h1 className={styles.heroTitle}>
          AI learned to think.<br />
          <span className={styles.highlight}>Now teach it to move.</span>
        </h1>
        <p className={styles.heroSubtitle}>
          You've built neural networks, trained models, deployed LLMs.<br />
          They live in servers. They respond in JSON.<br />
          <strong>This book gives AI a body.</strong>
        </p>
        <div className={styles.ctaGroup}>
          <Link className={styles.ctaPrimary} to="/docs/intro">
            Start the Journey
          </Link>
          <Link className={styles.ctaSecondary} to="/docs/Part-1-ROS2-Foundation">
            Jump to Part 1
          </Link>
        </div>
      </div>
    </section>
  );
}

function ManifestoSection() {
  return (
    <section className={styles.manifesto}>
      <div className={styles.manifestoContent}>
        <h2 className={styles.manifestoTitle}>The Old Way is Dead</h2>
        <div className={styles.manifestoGrid}>
          <div className={styles.oldWay}>
            <h3>Traditional CS Education</h3>
            <ul>
              <li>Theory before practice</li>
              <li>Simulations without purpose</li>
              <li>Robotics as an afterthought</li>
              <li>AI trapped in the cloud</li>
            </ul>
          </div>
          <div className={styles.newWay}>
            <h3>Physical AI</h3>
            <ul>
              <li>Build first, understand why</li>
              <li>Every simulation serves deployment</li>
              <li>Robotics as the destination</li>
              <li>AI that walks among us</li>
            </ul>
          </div>
        </div>
      </div>
    </section>
  );
}

function JourneySection() {
  const parts = [
    {
      number: '01',
      title: 'The Nervous System',
      subtitle: 'ROS 2',
      description: 'Nodes, topics, services. The middleware that connects sensors to actuators.',
    },
    {
      number: '02',
      title: 'The Digital Twin',
      subtitle: 'Gazebo & Unity',
      description: 'Physics simulation. Test in virtual worlds before reality.',
    },
    {
      number: '03',
      title: 'The AI Brain',
      subtitle: 'NVIDIA Isaac',
      description: 'GPU-accelerated perception. See, map, navigate autonomously.',
    },
    {
      number: '04',
      title: 'Voice to Action',
      subtitle: 'VLA Pipeline',
      description: 'Speak commands. LLMs plan. Robots execute.',
    },
  ];

  return (
    <section className={styles.journey}>
      <div className={styles.journeyContent}>
        <h2 className={styles.journeyTitle}>The Journey</h2>
        <p className={styles.journeySubtitle}>Four parts. Ten chapters. One autonomous humanoid.</p>
        <div className={styles.partsGrid}>
          {parts.map((part) => (
            <div key={part.number} className={styles.partCard}>
              <span className={styles.partNumber}>{part.number}</span>
              <h3 className={styles.partTitle}>{part.title}</h3>
              <span className={styles.partSubtitle}>{part.subtitle}</span>
              <p className={styles.partDescription}>{part.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function OutcomeSection() {
  return (
    <section className={styles.outcome}>
      <div className={styles.outcomeContent}>
        <h2 className={styles.outcomeTitle}>What You'll Build</h2>
        <div className={styles.outcomeDemo}>
          <div className={styles.commandFlow}>
            <div className={styles.flowStep}>
              <span className={styles.flowIcon}>voice</span>
              <p>"Pick up the red cup"</p>
            </div>
            <div className={styles.flowArrow}>to</div>
            <div className={styles.flowStep}>
              <span className={styles.flowIcon}>plan</span>
              <p>LLM decomposes task</p>
            </div>
            <div className={styles.flowArrow}>to</div>
            <div className={styles.flowStep}>
              <span className={styles.flowIcon}>act</span>
              <p>Robot executes</p>
            </div>
          </div>
        </div>
        <p className={styles.outcomeText}>
          A humanoid that hears, understands, navigates, and acts.<br />
          <strong>End to end. Voice to action.</strong>
        </p>
      </div>
    </section>
  );
}

function StartSection() {
  return (
    <section className={styles.start}>
      <div className={styles.startContent}>
        <h2 className={styles.startTitle}>Ready?</h2>
        <p className={styles.startText}>
          No robotics background required.<br />
          Just Python, curiosity, and the will to build.
        </p>
        <Link className={styles.startCta} to="/docs/intro">
          Begin Chapter 1
        </Link>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  return (
    <Layout
      title="From Digital AI to Physical Robotics"
      description="Physical AI & Humanoid Robotics - Teaching AI to move. A hands-on journey from neural networks to autonomous humanoids.">
      <main className={styles.main}>
        <HeroSection />
        <ManifestoSection />
        <JourneySection />
        <OutcomeSection />
        <StartSection />
      </main>
    </Layout>
  );
}
