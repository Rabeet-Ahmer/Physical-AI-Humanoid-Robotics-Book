import React from 'react';
import { authClient } from '../../lib/auth-client';
import { useHistory } from '@docusaurus/router';

export default function LogoutButton() {
  const history = useHistory();

  const handleLogout = async () => {
    await authClient.signOut({
        fetchOptions: {
            onSuccess: () => {
                history.push('/sign-in'); // Redirect to sign-in page after logout
            },
        },
    });
  };

  return (
    <button 
      onClick={handleLogout}
      className="button button--secondary button--sm"
    >
      Logout
    </button>
  );
}
