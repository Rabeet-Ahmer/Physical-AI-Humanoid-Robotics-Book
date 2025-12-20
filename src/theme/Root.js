import React from 'react';
import { useLocation, Redirect } from '@docusaurus/router';
import { authClient } from '../lib/auth-client';
import Layout from '@theme/Layout';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

export default function Root({children}) {
  const location = useLocation();
  const { data: session, isPending } = authClient.useSession();

  const currentPath = location.pathname;
  // Blocklist strategy: Only protect /docs/ URLs
  const isProtected = currentPath.startsWith('/docs');

  // 1. Server-Side / Static Build Protection
  // If we are on the server (SSG build time) and the route is protected,
  // we strictly return a placeholder. This ensures the static HTML *never* contains the book content.
  if (!ExecutionEnvironment.canUseDOM && isProtected) {
      return (
        <Layout>
            <div className="container margin-vert--xl" style={{textAlign: 'center', marginTop: '20vh'}}>
                <h2>Protected Content</h2>
                <p>Please sign in to view this page.</p>
            </div>
        </Layout>
      );
  }

  // 2. Client-Side Protection
  if (ExecutionEnvironment.canUseDOM && isProtected) {
    if (isPending) {
        return (
            <Layout>
                 <div className="container margin-vert--xl" style={{textAlign: 'center', marginTop: '20vh', display: 'flex', flexDirection: 'column', alignItems: 'center', gap: '20px'}}>
                    <h2>Verifying Access...</h2>
                    <div style={{
                        width: '40px',
                        height: '40px',
                        border: '4px solid #f3f3f3',
                        borderTop: '4px solid #25c2a0',
                        borderRadius: '50%',
                        animation: 'spin 1s linear infinite'
                    }}></div>
                    <style>{`
                        @keyframes spin {
                            0% { transform: rotate(0deg); }
                            100% { transform: rotate(360deg); }
                        }
                    `}</style>
                </div>
            </Layout>
        );
    }

    if (!session) {
        // Use declarative Redirect to avoid crash/race conditions with useEffect
        return <Redirect to="/sign-in" />;
    }
  }

  // Only render children if not protected OR (client-side + authenticated)
  return <>{children}</>;
}
