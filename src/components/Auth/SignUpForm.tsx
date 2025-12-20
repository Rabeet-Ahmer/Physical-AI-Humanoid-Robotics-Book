import React, { useState } from 'react';
import { authClient } from '../../lib/auth-client';
import { useHistory } from '@docusaurus/router';
import styles from './Auth.module.css';
import { AuthLayout } from './AuthLayout';
import Link from '@docusaurus/Link';
import { GoogleIcon } from './GoogleIcon';

export default function SignUpForm() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const history = useHistory();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError(null);

    await authClient.signUp.email({
      email,
      password,
      name,
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

  const handleGoogleSignUp = async () => {
    await authClient.signIn.social({
        provider: "google",
        callbackURL: "/",
    });
  };

  return (
    <AuthLayout title="Create Account" subtitle="Join us to get started">
      {error && <div className={styles.errorMessage}>{error}</div>}
      <form onSubmit={handleSubmit}>
        <div className={styles.formGroup}>
          <label htmlFor="name" className={styles.label}>Name</label>
          <input
            id="name"
            type="text"
            className={styles.input}
            value={name}
            onChange={(e) => setName(e.target.value)}
            required
            placeholder="John Doe"
          />
        </div>
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
            placeholder="Create a password"
          />
        </div>
        <button type="submit" disabled={loading} className={styles.submitButton}>
          {loading ? 'Signing Up...' : 'Sign Up'}
        </button>
      </form>

      <div className={styles.divider}>OR</div>

      <button
        type="button"
        onClick={handleGoogleSignUp}
        className={styles.googleButton}
      >
        <GoogleIcon />
        Sign up with Google
      </button>

      <div className={styles.footerLink}>
        Already have an account? <Link to="/sign-in" className={styles.link}>Sign in</Link>
      </div>
    </AuthLayout>
  );
}