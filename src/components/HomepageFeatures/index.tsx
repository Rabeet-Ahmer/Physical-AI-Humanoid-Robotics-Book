import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';
import { BentoGridContainer, BentoGridItem } from '../BentoGrid'; // Import BentoGrid components
import AnimatedContent from '../AnimatedContent'; // Import AnimatedContent component

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: JSX.Element;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Spec-Driven Development',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Learn to architect robotic systems before writing a single line of code.
        We emphasize <strong>System Design</strong>, <strong>Architecture</strong>, 
        and <strong>Verification</strong>.
      </>
    ),
  },
  {
    title: 'Sim-to-Real Workflow',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Master the <strong>Digital Twin</strong> concept. Build and test in 
        Gazebo/Isaac Sim, then deploy to physical hardware with confidence using ROS 2.
      </>
    ),
  },
  {
    title: 'Vision-Language-Action',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Integrate modern <strong>Foundation Models (VLAs)</strong> to give your 
        robot common sense and the ability to understand natural language commands.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className="text--center">
      <Svg className={styles.featureSvg} role="img" />
      <div className="text--center padding-horiz--md">
        <h3 className={styles.featureTitle}>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): JSX.Element {
  return (
    <section className={styles.features}>
      <div className="container">
        <h2 className="text--center">Key Features</h2> {/* Add a section title */}
        <BentoGridContainer>
          <BentoGridItem colSpan={2} variant="wide"> {/* Make one item span 2 columns */}
            <AnimatedContent animationType="slide-in-up" delay={0}>
              <Feature {...FeatureList[0]} />
            </AnimatedContent>
          </BentoGridItem>
          <BentoGridItem>
            <AnimatedContent animationType="slide-in-up" delay={100}>
              <Feature {...FeatureList[1]} />
            </AnimatedContent>
          </BentoGridItem>
          <BentoGridItem>
            <AnimatedContent animationType="slide-in-up" delay={200}>
              <Feature {...FeatureList[2]} />
            </AnimatedContent>
          </BentoGridItem>
        </BentoGridContainer>
      </div>
    </section>
  );
}