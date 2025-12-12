import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

interface FeatureBadgeProps {
  text: string;
  icon: string; // Emoji
}

const FeatureBadge: React.FC<FeatureBadgeProps> = ({ text, icon }) => {
  return (
    <span className={clsx(styles.featureBadge, 'badge')}>
      {icon} {text}
    </span>
  );
};

export default FeatureBadge;