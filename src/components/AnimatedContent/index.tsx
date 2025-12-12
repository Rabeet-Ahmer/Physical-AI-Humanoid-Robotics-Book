import React, { useRef, useEffect, useState } from 'react';
import clsx from 'clsx';
import { useInView } from 'react-intersection-observer';
import styles from './styles.module.css'; // For AnimatedContent specific styles
import globalAnimations from '../../styles/animations.module.css'; // For global animation classes

interface AnimatedContentProps {
  children: React.ReactNode;
  animationType?: string; // e.g., 'slide-in-up', 'fade-in'
  delay?: number; // in ms
  threshold?: number; // Intersection Observer threshold
  className?: string; // Additional classes for the wrapper div
}

const AnimatedContent: React.FC<AnimatedContentProps> = ({
  children,
  animationType = 'slide-in-up', // Default animation
  delay = 0,
  threshold = 0.1,
  className,
}) => {
  const { ref, inView } = useInView({
    triggerOnce: true, // Only trigger animation once
    threshold: threshold,
  });

  const [delayStyle, setDelayStyle] = useState({});

  useEffect(() => {
    if (delay > 0) {
      setDelayStyle({ transitionDelay: `${delay}ms` });
    }
  }, [delay]);

  const animationClass = globalAnimations[animationType] || globalAnimations['fade-in']; // Use specific or default

  return (
    <div
      ref={ref}
      className={clsx(
        styles.animatedContentWrapper,
        animationClass,
        inView && globalAnimations['is-visible'], // Class to trigger animation
        className
      )}
      style={delayStyle}
    >
      {children}
    </div>
  );
};

export default AnimatedContent;