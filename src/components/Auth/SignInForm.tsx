import React, { useState } from 'react';
import { authClient } from '../../lib/auth-client';
import { useHistory } from '@docusaurus/router';
import styles from './Auth.module.css';
import { AuthLayout } from './AuthLayout';
import Link from '@docusaurus/Link';
import { GoogleIcon } from './GoogleIcon';

export default function SignInForm() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const history = useHistory();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError(null);

    await authClient.signIn.email({
      email,
      password,
    }, {
      onSuccess: () => {
        setLoading(false);
        history.push('/');
      },
      onError: (ctx) => {
        setLoading(false);
        setError(ctx.error.message);
      },
    });
  };

  const handleGoogleSignIn = async () => {
    await authClient.signIn.social({
        provider: "google",
        callbackURL: "/",
    });
  };

  return (
    <AuthLayout title="Welcome Back" subtitle="Sign in to your account to continue">
      {error && <div className={styles.errorMessage}>{error}</div>}
      <form onSubmit={handleSubmit}>
        <div className={styles.formGroup}>
          <label htmlFor="email" className={styles.label}>Email</label>
          <input
            id="email"
            type="email"
            className={styles.input}
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
            placeholder="name@example.com"
          />
        </div>
        <div className={styles.formGroup}>
          <label htmlFor="password" className={styles.label}>Password</label>
          <input
            id="password"
            type="password"
            className={styles.input}
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
            placeholder="Enter your password"
          />
        </div>
        <button type="submit" disabled={loading} className={styles.submitButton}>
          {loading ? 'Signing In...' : 'Sign In'}
        </button>
      </form>

      <div className={styles.divider}>OR</div>

      <button
        type="button"
        onClick={handleGoogleSignIn}
        className={styles.googleButton}
      >
        <GoogleIcon />
        Sign in with Google
      </button>
      
      <div className={styles.footerLink}>
        Don't have an account? <Link to="/sign-up" className={styles.link}>Sign up</Link>
      </div>
    </AuthLayout>
  );
}
