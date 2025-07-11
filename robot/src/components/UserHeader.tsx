import React from 'react';
import { useNavigate } from 'react-router-dom';

// Simple user management without complex context
const getUserName = (): string => {
  return localStorage.getItem('robotUser') || 'Guest';
};

const setUserName = (name: string): void => {
  localStorage.setItem('robotUser', name);
};

const logout = (): void => {
  localStorage.setItem('robotUser', 'Guest');
  window.location.href = '/'; // Go back to face recognition
};

// Simple User Header Component
interface UserHeaderProps {
  showLogout?: boolean;
}

export const UserHeader: React.FC<UserHeaderProps> = ({ showLogout = true }) => {
  const navigate = useNavigate();
  const currentUser = getUserName();
  const isGuest = currentUser === 'Guest';

  const handleLogout = () => {
    logout();
  };

  const headerStyle = {
    position: 'fixed' as const,
    top: '1rem',
    left: '1rem',
    right: '1rem',
    zIndex: 1000,
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: '1rem 2rem',
    background: 'rgba(255, 255, 255, 0.95)',
    backdropFilter: 'blur(10px)',
    borderRadius: '12px',
    boxShadow: '0 4px 8px rgba(0, 0, 0, 0.1)',
    border: '1px solid rgba(255, 255, 255, 0.2)'
  };

  const greetingStyle = {
    fontSize: '1.2rem',
    fontWeight: '600' as const,
    color: '#2d3748',
    display: 'flex',
    alignItems: 'center',
    gap: '0.5rem'
  };

  const logoutButtonStyle = {
    padding: '0.5rem 1rem',
    borderRadius: '8px',
    border: 'none',
    cursor: 'pointer',
    background: 'linear-gradient(135deg, #f56565, #e53e3e)',
    color: 'white',
    fontSize: '0.9rem',
    fontWeight: '600' as const,
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
        <span style={{ fontSize: '1.5rem' }}>
          {isGuest ? '👤' : '👋'}
        </span>
        <span>Hi, {currentUser}!</span>
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
            onClick={handleLogout}
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

// Export simple user management functions
export { getUserName, setUserName, logout };
