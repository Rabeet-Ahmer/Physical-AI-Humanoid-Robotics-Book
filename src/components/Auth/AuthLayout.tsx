import React, { ReactNode } from 'react';
import styles from './Auth.module.css';

interface AuthLayoutProps {
  children: ReactNode;
  title: string;
  subtitle?: string;
}

export const AuthLayout = ({ children, title, subtitle }: AuthLayoutProps) => {
  return (
    <div className={styles.authContainer}>
      <div className={styles.authCard}>
        <h1 className={styles.authTitle}>{title}</h1>
        {subtitle && <p className={styles.authSubtitle}>{subtitle}</p>}
        {children}
      </div>
    </div>
  );
};
