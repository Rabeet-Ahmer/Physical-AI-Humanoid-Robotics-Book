import React from 'react';
import clsx from 'clsx';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';
import FeatureBadge from '../FeatureBadge'; // Import FeatureBadge component
import CTAButton from '../CTAButton'; // Import CTAButton component

export default function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className={clsx('container', styles.heroContainer)}>
        <div className="row">
          {/* Left Column for Image */}
          <div className={clsx('col col--6', styles.heroImageColumn)}>
            {/* Placeholder for Hero Image - T010 */}
            <img src="/img/hero-book-cover.jpg" alt="AI Native Robotics Book Cover" className={styles.heroImage} />
          </div>

          {/* Right Column for Text Content */}
          <div className={clsx('col col--6', styles.heroContentColumn)}>
            {/* Top Label Badge - T011 */}
            <span className={styles.topBadge}>PHYSICAL AI BOOK</span>

            {/* Main Title - T012 */}
            <h1 className="hero__title">AI Native Robotics</h1>

            {/* Subtitle - T013 */}
            <p className="hero__subtitle">Learn Physical AI & Humanoid Robotics the modern way</p>

            {/* Feature Badges Row - T016 */}
            <div className={styles.featureBadges}>
                <FeatureBadge text="Open Source" icon="✨" />
                <FeatureBadge text="Co-Learning with AI" icon="🤝" />
                <FeatureBadge text="Spec-Driven Development" icon="🎯" />
            </div>

            {/* Call-to-Action Buttons - T019 */}
            <div className={styles.buttons}>
              <CTAButton
                text="Start Reading →"
                to="/docs/intro"
                styleType="primary"
              />
              <CTAButton
                text="Explore Project"
                to="/docs/intro" // Link to docs intro as a placeholder
                styleType="secondary"
                icon="🎓"
              />
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}