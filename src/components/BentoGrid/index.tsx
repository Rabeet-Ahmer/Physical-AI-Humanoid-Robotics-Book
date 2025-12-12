import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

interface BentoGridContainerProps {
  children: React.ReactNode;
  className?: string;
}

export const BentoGridContainer: React.FC<BentoGridContainerProps> = ({ children, className }) => {
  return (
    <div className={clsx(styles.bentoGridContainer, className)}>
      {children}
    </div>
  );
};

interface BentoGridItemProps {
  children: React.ReactNode;
  className?: string;
  colSpan?: number;
  rowSpan?: number;
  variant?: 'default' | 'tall' | 'wide' | 'large'; // Custom variants for styling
}

export const BentoGridItem: React.FC<BentoGridItemProps> = ({
  children,
  className,
  colSpan,
  rowSpan,
  variant,
}) => {
  const itemStyle: React.CSSProperties = {};
  if (colSpan) {
    itemStyle.gridColumn = `span ${colSpan}`;
  }
  if (rowSpan) {
    itemStyle.gridRow = `span ${rowSpan}`;
  }

  return (
    <div
      className={clsx(
        styles.bentoGridItem,
        variant && styles[variant], // Apply variant specific styles
        className
      )}
      style={itemStyle}
    >
      {children}
    </div>
  );
};
