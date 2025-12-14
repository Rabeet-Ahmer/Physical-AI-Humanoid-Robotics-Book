import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';
import { BentoGridContainer, BentoGridItem } from '../BentoGrid'; // Import BentoGrid components

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
    <Link to={link} className={styles.moduleLink}>
      <div className="card padding--lg h-100">
        <div className="card__header">
          <h3>{title}</h3>
        </div>
        <div className="card__body">
          <p>{description}</p>
        </div>
        <div className="card__footer">
          <span className="button button--secondary button--block">Start Module</span>
        </div>
      </div>
    </Link>
  );
}

export default function HomepageCurriculum(): JSX.Element {
  return (
    <section className={styles.curriculumSection}>
      <div className="container">
        <h2 className="text--center margin-bottom--lg">Curriculum Roadmap</h2>
        <BentoGridContainer>
          {/* Example bento layout: one item spans 2 columns */}
          <BentoGridItem colSpan={2} variant="wide">
            <Module {...ModuleList[0]} />
          </BentoGridItem>
          <BentoGridItem>
            <Module {...ModuleList[1]} />
          </BentoGridItem>
          <BentoGridItem>
            <Module {...ModuleList[2]} />
          </BentoGridItem>
          <BentoGridItem>
            <Module {...ModuleList[3]} />
          </BentoGridItem>
        </BentoGridContainer>
      </div>
    </section>
  );
}