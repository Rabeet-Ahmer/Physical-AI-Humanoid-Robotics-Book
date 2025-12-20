import React from 'react';
import { authClient } from '../lib/auth-client';
import { Redirect } from '@docusaurus/router';

export default function ProtectedRoute({ children }: { children: React.ReactNode }) {
  const { data: session, isPending, error } = authClient.useSession();

  if (isPending) {
    return <div>Loading...</div>; // Replace with a nicer spinner if desired
  }

  if (error || !session) {
    return <Redirect to="/sign-in" />;
  }

  return <>{children}</>;
}
