import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

interface CTAButtonProps {
  text: string;
  to: string;
  styleType: 'primary' | 'secondary';
  icon?: string; // Optional emoji/icon
}

const CTAButton: React.FC<CTAButtonProps> = ({ text, to, styleType, icon }) => {
  return (
    <Link
      className={clsx('button', styles.ctaButton, {
        [styles.primaryButton]: styleType === 'primary',
        [styles.secondaryButton]: styleType === 'secondary',
      })}
      to={to}>
      {text} {icon && <span className={styles.buttonIcon}>{icon}</span>}
    </Link>
  );
};

export default CTAButton;