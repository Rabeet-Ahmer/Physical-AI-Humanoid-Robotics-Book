import React, { JSX } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

type ModuleItem = {
  title: string;
  link: string;
  description: string;
};

const ModuleList: ModuleItem[] = [
  {
    title: 'Module 1: The Nervous System',
    link: '/docs/module-01-nervous-system/overview',
    description: 'Master ROS 2 nodes, topics, and services to build the communication infrastructure.',
  },
  {
    title: 'Module 2: The Digital Twin',
    link: '/docs/module-02-digital-twin/course-introduction',
    description: 'Simulate physics, sensors, and environments using Gazebo and Unity.',
  },
  {
    title: 'Module 3: The AI Brain',
    link: '/docs/module-03-ai-brain/course-introduction', // Corrected link based on existing structure
    description: 'Implement perception, navigation (Nav2), and SLAM using NVIDIA Isaac Sim.',
  },
  {
    title: 'Module 4: VLA (Vision-Language-Action)',
    link: '/docs/module-04-vla/course-introduction',
    description: 'The capstone: Integrate LLMs/VLAs for embodied intelligence and natural language control.',
  },
];

function Module({title, link, description}: ModuleItem) {
  return (
    <div className={clsx('col col--6 margin-bottom--lg')}>
      <Link to={link} className={styles.moduleLink}>
        <div className="card padding--lg h-100">
          <div className="card__header">
            <h3 className={styles.moduleTitle}>{title}</h3>
          </div>
          <div className="card__body">
            <p>{description}</p>
          </div>
          <div className="card__footer">
            <span className="button button--secondary button--block">Start Module</span>
          </div>
        </div>
      </Link>
    </div>
  );
}

export default function HomepageCurriculum(): JSX.Element {
  return (
    <section className={styles.curriculumSection}>
      <div className="container">
        <h2 className={clsx("text--center", styles.sectionTitle)}>Curriculum Roadmap</h2>
        <div className="row">
          {ModuleList.map((props, idx) => (
            <Module key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}