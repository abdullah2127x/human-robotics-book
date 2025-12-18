import type { ReactNode } from 'react';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';

import styles from './index.module.css';

function HeroSection() {
  return (
    <section className={styles.hero}>
      <div className={styles.heroBackground}>
        <div className={styles.gridLines}></div>
        <div className={styles.glowOrb}></div>
        <div className={styles.glowOrb2}></div>
      </div>
      <div className={styles.heroContent}>
        <div className={styles.badge}>
          <span className={styles.badgeIcon}>◈</span>
          <span>Physical AI & Humanoid Robotics</span>
        </div>
        <h1 className={styles.heroTitle}>
          <span className={styles.titleLine}>AI learned to think.</span>
          <span className={styles.titleLine}>
            <span className={styles.highlight}>Now teach it to move.</span>
          </span>
        </h1>
        <p className={styles.heroSubtitle}>
          Master the complete pipeline from ROS 2 foundations to voice-controlled
          humanoid robots. Build autonomous systems that perceive, reason, and act
          in the physical world.
        </p>
        <div className={styles.stats}>
          <div className={styles.stat}>
            <span className={styles.statNumber}>10</span>
            <span className={styles.statLabel}>Chapters</span>
          </div>
          <div className={styles.statDivider}></div>
          <div className={styles.stat}>
            <span className={styles.statNumber}>80+</span>
            <span className={styles.statLabel}>Lessons</span>
          </div>
          <div className={styles.statDivider}></div>
          <div className={styles.stat}>
            <span className={styles.statNumber}>4</span>
            <span className={styles.statLabel}>Parts</span>
          </div>
        </div>
        <div className={styles.ctaGroup}>
          <Link className={styles.ctaPrimary} to="/docs/intro">
            <span>Start Learning</span>
            <svg className={styles.ctaArrow} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M5 12h14M12 5l7 7-7 7"/>
            </svg>
          </Link>
          <Link className={styles.ctaSecondary} to="/docs/Part-1-ROS2-Foundation">
            View Curriculum
          </Link>
        </div>
      </div>
      <div className={styles.scrollIndicator}>
        <span>Scroll to explore</span>
        <div className={styles.scrollLine}></div>
      </div>
    </section>
  );
}

function TechStackSection() {
  const technologies = [
    { name: 'ROS 2', icon: '🤖' },
    { name: 'Python', icon: '🐍' },
    { name: 'Gazebo', icon: '🌍' },
    { name: 'Unity', icon: '🎮' },
    { name: 'Isaac Sim', icon: '⚡' },
    { name: 'Nav2', icon: '🧭' },
    { name: 'VSLAM', icon: '👁️' },
    { name: 'LLMs', icon: '🧠' },
  ];

  return (
    <section className={styles.techStack}>
      <div className={styles.techStackContent}>
        <p className={styles.techLabel}>Technologies You'll Master</p>
        <div className={styles.techScroll}>
          <div className={styles.techTrack}>
            {[...technologies, ...technologies].map((tech, index) => (
              <div key={index} className={styles.techItem}>
                <span className={styles.techIcon}>{tech.icon}</span>
                <span className={styles.techName}>{tech.name}</span>
              </div>
            ))}
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
      title: 'The Robotic Nervous System',
      subtitle: 'ROS 2 Foundation',
      description: 'Master nodes, topics, services, and actions. Build the communication backbone that connects every sensor and actuator.',
      chapters: ['ROS 2 Core Concepts', 'Python Development with rclpy', 'URDF & Humanoid Modeling'],
      color: '#00ff88',
      link: '/docs/Part-1-ROS2-Foundation',
    },
    {
      number: '02',
      title: 'The Digital Twin',
      subtitle: 'Simulation Mastery',
      description: 'Create high-fidelity virtual environments. Test algorithms safely before deploying to real hardware.',
      chapters: ['Gazebo Physics Simulation', 'Unity for HRI', 'Sensor Simulation'],
      color: '#00d4ff',
      link: '/docs/Part-2-Digital-Twin',
    },
    {
      number: '03',
      title: 'The AI Brain',
      subtitle: 'Advanced Perception',
      description: 'Harness GPU-accelerated AI for perception. Enable your robot to see, map, and navigate autonomously.',
      chapters: ['NVIDIA Isaac Sim', 'Visual SLAM', 'Nav2 Navigation'],
      color: '#ff6b6b',
      link: '/docs/Part-3-Advanced-Simulation-Perception',
    },
    {
      number: '04',
      title: 'Voice to Action',
      subtitle: 'VLA Pipeline',
      description: 'Connect language models to robot control. Speak commands, watch robots execute complex tasks.',
      chapters: ['Speech Recognition', 'LLM Task Planning', 'Action Execution'],
      color: '#ffd93d',
      link: '/docs/Part-4-Vision-Language-Action',
    },
  ];

  return (
    <section className={styles.journey}>
      <div className={styles.journeyContent}>
        <div className={styles.sectionHeader}>
          <span className={styles.sectionTag}>The Learning Path</span>
          <h2 className={styles.sectionTitle}>Your Journey to Physical AI</h2>
          <p className={styles.sectionSubtitle}>
            Four comprehensive parts take you from fundamentals to building
            voice-controlled autonomous humanoids.
          </p>
        </div>
        <div className={styles.partsGrid}>
          {parts.map((part, index) => (
            <Link
              key={part.number}
              to={part.link}
              className={styles.partCard}
              style={{ '--card-accent': part.color } as React.CSSProperties}
            >
              <div className={styles.partHeader}>
                <span className={styles.partNumber}>{part.number}</span>
                <span className={styles.partSubtitle}>{part.subtitle}</span>
              </div>
              <h3 className={styles.partTitle}>{part.title}</h3>
              <p className={styles.partDescription}>{part.description}</p>
              <div className={styles.partChapters}>
                {part.chapters.map((chapter, i) => (
                  <span key={i} className={styles.chapterTag}>{chapter}</span>
                ))}
              </div>
              <div className={styles.partFooter}>
                <span className={styles.exploreLink}>
                  Explore Part {part.number}
                  <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <path d="M5 12h14M12 5l7 7-7 7"/>
                  </svg>
                </span>
              </div>
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
}

function FeaturesSection() {
  const features = [
    {
      icon: '🎯',
      title: 'Project-Based Learning',
      description: 'Every chapter builds toward a working capstone project. Learn by doing, not just reading.',
    },
    {
      icon: '🔧',
      title: 'Production-Ready Code',
      description: 'All examples use industry-standard tools and practices. Code that actually runs on real robots.',
    },
    {
      icon: '📊',
      title: 'Progressive Complexity',
      description: 'Start simple, build complexity. Each lesson builds on the last with clear prerequisites.',
    },
    {
      icon: '🌐',
      title: 'End-to-End Pipeline',
      description: 'From sensor data to motor commands. Understand the complete robotics software stack.',
    },
    {
      icon: '🤝',
      title: 'Human-Robot Interaction',
      description: 'Build robots that communicate naturally through voice, gestures, and intuitive interfaces.',
    },
    {
      icon: '⚡',
      title: 'GPU Acceleration',
      description: 'Leverage NVIDIA Isaac for real-time perception. Train and deploy AI at robot speed.',
    },
  ];

  return (
    <section className={styles.features}>
      <div className={styles.featuresContent}>
        <div className={styles.sectionHeader}>
          <span className={styles.sectionTag}>Why This Book</span>
          <h2 className={styles.sectionTitle}>Built for Real-World Robotics</h2>
          <p className={styles.sectionSubtitle}>
            Not another theory-heavy textbook. This is a hands-on guide to building
            robots that work.
          </p>
        </div>
        <div className={styles.featuresGrid}>
          {features.map((feature, index) => (
            <div key={index} className={styles.featureCard}>
              <span className={styles.featureIcon}>{feature.icon}</span>
              <h3 className={styles.featureTitle}>{feature.title}</h3>
              <p className={styles.featureDescription}>{feature.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function DemoSection() {
  return (
    <section className={styles.demo}>
      <div className={styles.demoContent}>
        <div className={styles.sectionHeader}>
          <span className={styles.sectionTag}>The End Goal</span>
          <h2 className={styles.sectionTitle}>Voice to Action Pipeline</h2>
          <p className={styles.sectionSubtitle}>
            By the end of this book, you'll build a complete system that turns
            natural language into robot actions.
          </p>
        </div>
        <div className={styles.pipelineDemo}>
          <div className={styles.pipelineStep}>
            <div className={styles.stepIcon}>🎤</div>
            <div className={styles.stepContent}>
              <span className={styles.stepLabel}>Input</span>
              <p className={styles.stepText}>"Pick up the red cup and place it on the table"</p>
            </div>
          </div>
          <div className={styles.pipelineArrow}>
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M5 12h14M12 5l7 7-7 7"/>
            </svg>
          </div>
          <div className={styles.pipelineStep}>
            <div className={styles.stepIcon}>🧠</div>
            <div className={styles.stepContent}>
              <span className={styles.stepLabel}>LLM Planning</span>
              <p className={styles.stepText}>Task decomposition → Action sequence → Motion planning</p>
            </div>
          </div>
          <div className={styles.pipelineArrow}>
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M5 12h14M12 5l7 7-7 7"/>
            </svg>
          </div>
          <div className={styles.pipelineStep}>
            <div className={styles.stepIcon}>🤖</div>
            <div className={styles.stepContent}>
              <span className={styles.stepLabel}>Execution</span>
              <p className={styles.stepText}>Navigate → Grasp → Transport → Place</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function PrerequisitesSection() {
  return (
    <section className={styles.prerequisites}>
      <div className={styles.prerequisitesContent}>
        <div className={styles.prereqCard}>
          <h3 className={styles.prereqTitle}>What You Need</h3>
          <ul className={styles.prereqList}>
            <li>
              <span className={styles.checkIcon}>✓</span>
              <span>Basic Python programming experience</span>
            </li>
            <li>
              <span className={styles.checkIcon}>✓</span>
              <span>Familiarity with Linux command line</span>
            </li>
            <li>
              <span className={styles.checkIcon}>✓</span>
              <span>Understanding of basic ML concepts (helpful but not required)</span>
            </li>
            <li>
              <span className={styles.checkIcon}>✓</span>
              <span>Curiosity and willingness to experiment</span>
            </li>
          </ul>
        </div>
        <div className={styles.prereqCard}>
          <h3 className={styles.prereqTitle}>What You'll Gain</h3>
          <ul className={styles.prereqList}>
            <li>
              <span className={styles.starIcon}>★</span>
              <span>Deep understanding of robotics middleware (ROS 2)</span>
            </li>
            <li>
              <span className={styles.starIcon}>★</span>
              <span>Ability to build and simulate humanoid robots</span>
            </li>
            <li>
              <span className={styles.starIcon}>★</span>
              <span>Skills in AI-powered perception and navigation</span>
            </li>
            <li>
              <span className={styles.starIcon}>★</span>
              <span>Complete voice-to-action system implementation</span>
            </li>
          </ul>
        </div>
      </div>
    </section>
  );
}

function CTASection() {
  return (
    <section className={styles.cta}>
      <div className={styles.ctaContent}>
        <div className={styles.ctaGlow}></div>
        <h2 className={styles.ctaTitle}>Ready to Build the Future?</h2>
        <p className={styles.ctaText}>
          Start your journey from digital AI to physical robotics.
          No prior robotics experience required.
        </p>
        <div className={styles.ctaButtons}>
          <Link className={styles.ctaButtonPrimary} to="/docs/intro">
            <span>Begin Your Journey</span>
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M5 12h14M12 5l7 7-7 7"/>
            </svg>
          </Link>
          <Link className={styles.ctaButtonSecondary} to="/docs/Part-1-ROS2-Foundation/ros2-nodes-topics-services">
            Jump to Chapter 1
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  return (
    <Layout
      title="From Digital AI to Physical Robotics"
      description="Physical AI & Humanoid Robotics - Master the complete pipeline from ROS 2 foundations to voice-controlled humanoid robots. Build autonomous systems that perceive, reason, and act in the physical world.">
      <main className={styles.main}>
        <HeroSection />
        <TechStackSection />
        <JourneySection />
        <FeaturesSection />
        <DemoSection />
        <PrerequisitesSection />
        <CTASection />
      </main>
    </Layout>
  );
}
