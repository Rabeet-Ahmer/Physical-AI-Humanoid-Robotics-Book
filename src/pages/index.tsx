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
            Start Learning ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className={clsx('col col--4')}>
            <div className="text--center padding-horiz--md">
              <Heading as="h3">Focus & Theme</Heading>
              <p>
                Physical AI is not just software—it's about the interplay between code,
                physics, and hardware. Master the "Software 2.0" stack for robotics.
              </p>
            </div>
          </div>
          <div className={clsx('col col--4')}>
            <div className="text--center padding-horiz--md">
              <Heading as="h3">Goal</Heading>
              <p>
                Build a physical AI agent capable of manipulating the world.
                From Digital Twin simulation to Real-World deployment on edge hardware.
              </p>
            </div>
          </div>
          <div className={clsx('col col--4')}>
            <div className="text--center padding-horiz--md">
              <Heading as="h3">Why It Matters</Heading>
              <p>
                Embodied Intelligence is the next frontier. Move beyond chatbots to
                agents that can perceive, reason, and act in the physical world.
              </p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function LearningOutcomes() {
  return (
    <section className="padding-vert--xl">
      <div className="container">
        <Heading as="h2" className="text--center margin-bottom--lg">
          Learning Outcomes
        </Heading>
        <div className="row">
           <div className="col col--8 col--offset-2">
             <ul>
               <li>Architect ROS 2 nodes on Linux.</li>
               <li>Create accurate URDF/MJCF Digital Twins.</li>
               <li>Train Imitation Learning policies (ACT/Diffusion).</li>
               <li>Deploy VLA models to Jetson Orin Edge hardware.</li>
             </ul>
           </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Course">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <LearningOutcomes />
      </main>
    </Layout>
  );
}