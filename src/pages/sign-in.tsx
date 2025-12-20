import React from 'react';
import Layout from '@theme/Layout';
import SignInForm from '../components/Auth/SignInForm';

export default function SignIn() {
  return (
    <Layout title="Sign In" description="Log in to your account">
      <SignInForm />
    </Layout>
  );
}