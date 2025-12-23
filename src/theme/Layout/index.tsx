import React, {type ReactNode, useEffect} from 'react';
import Layout from '@theme-original/Layout';
import type LayoutType from '@theme/Layout';
import type {WrapperProps} from '@docusaurus/types';
import { useLocation, Redirect, useHistory } from '@docusaurus/router';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import { authClient } from '../../lib/auth-client';

import { ChatProvider } from '../../components/ChatWidget/ChatContext';
import ChatIcon from '../../components/ChatWidget/ChatIcon';
import ChatPanel from '../../components/ChatWidget/ChatPanel';

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): ReactNode {
  const location = useLocation();
  const history = useHistory();
  const { data: session, isPending } = authClient.useSession();

  const currentPath = location.pathname;
  // Blocklist strategy: Only protect /docs/ URLs
  const isProtected = currentPath.startsWith('/docs');

  let content = (props as { children?: ReactNode }).children;

  // 1. Server-Side / Static Build Protection
  if (!ExecutionEnvironment.canUseDOM && isProtected) {
    content = (
      <div className="container margin-vert--xl" style={{textAlign: 'center', marginTop: '20vh'}}>
          <h2>Protected Content</h2>
          <p>Please sign in to view this page.</p>
          <a href="/sign-in" className="button button--primary">Sign In</a>
      </div>
    );
  }
  // 2. Client-Side Protection
  else if (ExecutionEnvironment.canUseDOM && isProtected) {
    if (isPending) {
        content = (
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
        );
    } else if (!session) {
        // Use declarative Redirect
        return <Redirect to="/sign-in" />;
    }
  }
  
  // Backup effect for redirection if the declarative Redirect somehow fails or during transitions
  // useEffect(() => {
  //   if (ExecutionEnvironment.canUseDOM && isProtected && !isPending && !session) {
  //      history.replace('/sign-in');
  //   }
  // }, [isProtected, isPending, session, history]);

  return (
    <ChatProvider>
      <Layout {...props}>
        {content}
      </Layout>
      <ChatIcon />
      <ChatPanel />
    </ChatProvider>
  );
}