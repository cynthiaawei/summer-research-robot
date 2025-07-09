import React, { createContext, useContext, useState, useEffect } from 'react';

interface UserContextType {
  currentUser: string;
  setCurrentUser: (user: string) => void;
  isGuest: boolean;
  logout: () => void;
}

const UserContext = createContext<UserContextType | undefined>(undefined);

interface UserProviderProps {
  children: React.ReactNode;
}

export const UserProvider: React.FC<UserProviderProps> = ({ children }) => {
  const [currentUser, setCurrentUserState] = useState<string>('');

  useEffect(() => {
    // Load user from localStorage on app start
    const savedUser = localStorage.getItem('robotUser');
    if (savedUser) {
      setCurrentUserState(savedUser);
    }
  }, []);

  const setCurrentUser = (user: string) => {
    setCurrentUserState(user);
    localStorage.setItem('robotUser', user);
  };

  const logout = () => {
    setCurrentUserState('');
    localStorage.removeItem('robotUser');
    // Navigate back to face recognition
    window.location.href = '/';
  };

  const isGuest = currentUser === 'Guest' || currentUser === '';

  return (
    <UserContext.Provider value={{
      currentUser,
      setCurrentUser,
      isGuest,
      logout
    }}>
      {children}
    </UserContext.Provider>
  );
};

export const useUser = () => {
  const context = useContext(UserContext);
  if (context === undefined) {
    throw new Error('useUser must be used within a UserProvider');
  }
  return context;
};

// User Header Component - shows "Hi {name}" on every page
interface UserHeaderProps {
  showLogout?: boolean;
}

export const UserHeader: React.FC<UserHeaderProps> = ({ showLogout = true }) => {
  const { currentUser, isGuest, logout } = useUser();

  if (!currentUser) {
    return null;
  }

  const headerStyle = {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: '1rem 2rem',
    background: 'rgba(255, 255, 255, 0.9)',
    backdropFilter: 'blur(10px)',
    borderRadius: '12px',
    marginBottom: '1rem',
    boxShadow: '0 4px 8px rgba(0, 0, 0, 0.1)',
    border: '1px solid rgba(255, 255, 255, 0.2)'
  };

  const greetingStyle = {
    fontSize: '1.2rem',
    fontWeight: '600',
    color: '#2d3748',
    display: 'flex',
    alignItems: 'center',
    gap: '0.5rem'
  };

  const userIconStyle = {
    fontSize: '1.5rem'
  };

  const logoutButtonStyle = {
    padding: '0.5rem 1rem',
    borderRadius: '8px',
    border: 'none',
    cursor: 'pointer',
    background: 'linear-gradient(135deg, #f56565, #e53e3e)',
    color: 'white',
    fontSize: '0.9rem',
    fontWeight: '600',
    transition: 'all 0.3s ease'
  };

  const statusStyle = {
    fontSize: '0.9rem',
    color: '#718096',
    display: 'flex',
    alignItems: 'center',
    gap: '0.5rem'
  };

  return (
    <div style={headerStyle}>
      <div style={greetingStyle}>
        <span style={userIconStyle}>
          {isGuest ? '👤' : '👋'}
        </span>
        <span>
          {isGuest ? 'Welcome, Guest!' : `Hi, ${currentUser}!`}
        </span>
      </div>
      
      <div style={{ display: 'flex', alignItems: 'center', gap: '1rem' }}>
        <div style={statusStyle}>
          <span style={{
            width: '8px',
            height: '8px',
            borderRadius: '50%',
            backgroundColor: isGuest ? '#ed8936' : '#48bb78',
            display: 'inline-block'
          }}></span>
          {isGuest ? 'Guest Mode' : 'Registered User'}
        </div>
        
        {showLogout && (
          <button
            onClick={logout}
            style={logoutButtonStyle}
            onMouseEnter={(e) => {
              e.currentTarget.style.transform = 'translateY(-2px)';
              e.currentTarget.style.boxShadow = '0 4px 8px rgba(245, 101, 101, 0.4)';
            }}
            onMouseLeave={(e) => {
              e.currentTarget.style.transform = 'translateY(0)';
              e.currentTarget.style.boxShadow = 'none';
            }}
            title="Return to face recognition"
          >
            🚪 Logout
          </button>
        )}
      </div>
    </div>
  );
};