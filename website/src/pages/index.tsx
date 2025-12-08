import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)} role="banner">
      <div className="container">
        <div className="text--center">
          <Heading as="h1" className="hero__title" aria-level="1">
            Physical AI & Humanoid Robotics
          </Heading>
          <p className="hero__subtitle" aria-label="Tagline: Bridging Digital Minds to Physical Bodies">
            Bridging Digital Minds to Physical Bodies
          </p>
          <div className={styles.buttons} role="group" aria-label="Primary navigation buttons">
            <Link
              className="button button--secondary button--lg margin-right--md"
              to="/docs/intro"
              aria-label="Start learning with Module 1">
              Start Learning (Module 1)
            </Link>
            <Link
              className="button button--outline button--lg"
              to="#"
              aria-label="Chat with book feature (placeholder)">
              Chat with Book (Placeholder)
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="Bridging Digital Minds to Physical Bodies">
      <HomepageHeader />
      <main>
        <section className={styles.features} aria-labelledby="features-heading">
          <h2 id="features-heading" className="visually-hidden">Key Learning Areas</h2>
          <div className="container padding-vert--lg">
            <div className="row">
              <div className="col col--4 padding--md">
                <div className="text--center padding--sm"
                     style={{border: '1px solid var(--ifm-color-emphasis-300)', borderRadius: '8px'}}
                     role="region"
                     aria-labelledby="feature1-heading"
                     aria-describedby="feature1-description">
                  <Heading as="h3" id="feature1-heading">AI-First Pedagogy</Heading>
                  <p id="feature1-description">Learn about using AI tools to accelerate robotics development and understanding</p>
                </div>
              </div>
              <div className="col col--4 padding--md">
                <div className="text--center padding--sm"
                     style={{border: '1px solid var(--ifm-color-emphasis-300)', borderRadius: '8px'}}
                     role="region"
                     aria-labelledby="feature2-heading"
                     aria-describedby="feature2-description">
                  <Heading as="h3" id="feature2-heading">The 4-Layer Framework</Heading>
                  <p id="feature2-description">Structured learning path from manual foundation to spec-driven integration</p>
                </div>
              </div>
              <div className="col col--4 padding--md">
                <div className="text--center padding--sm"
                     style={{border: '1px solid var(--ifm-color-emphasis-300)', borderRadius: '8px'}}
                     role="region"
                     aria-labelledby="feature3-heading"
                     aria-describedby="feature3-description">
                  <Heading as="h3" id="feature3-heading">Physical Hardware & Digital Twins</Heading>
                  <p id="feature3-description">Hands-on experience with real robotics and digital simulation</p>
                </div>
              </div>
            </div>

            {/* Additional section to support progressive complexity */}
            <div className="row">
              <div className="col col--12 padding-horiz--md">
                <div className="text--center padding--md"
                     style={{border: '2px dashed var(--ifm-color-emphasis-300)', borderRadius: '8px', backgroundColor: 'var(--ifm-color-emphasis-100)'}}>
                  <Heading as="h3">Progressive Learning Path</Heading>
                  <p>Our curriculum is designed with cognitive load management in mind, starting with foundational concepts and gradually building to advanced applications.</p>
                  <div className="row" style={{marginTop: '1rem'}}>
                    <div className="col col--4">
                      <strong>Foundation</strong>
                      <p>Basic concepts and manual operations</p>
                    </div>
                    <div className="col col--4">
                      <strong>Integration</strong>
                      <p>AI-assisted development and tools</p>
                    </div>
                    <div className="col col--4">
                      <strong>Mastery</strong>
                      <p>Advanced applications and research</p>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}

