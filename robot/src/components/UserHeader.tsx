import React, { useState, useEffect } from 'react';
import { useNavigate } from 'react-router-dom';

// Simple user management with better error handling
const getUserName = (): string => {
  try {
    return localStorage.getItem('robotUser') || 'Guest';
  } catch (error) {
    console.warn('LocalStorage not available:', error);
    return 'Guest';
  }
};

const setUserName = (name: string): void => {
  try {
    localStorage.setItem('robotUser', name);
    // Dispatch event immediately after setting
    if (typeof window !== 'undefined') {
      window.dispatchEvent(new CustomEvent('userChanged'));
    }
  } catch (error) {
    console.warn('LocalStorage not available:', error);
  }
};

const logout = (): void => {
  try {
    // FIXED: Set to Guest first, then dispatch event
    localStorage.setItem('robotUser', 'Guest');
    
    // Dispatch custom event for same-tab updates
    if (typeof window !== 'undefined') {
      window.dispatchEvent(new CustomEvent('userChanged'));
    }
    
    // FIXED: Small delay to ensure state updates, then navigate
    setTimeout(() => {
      if (typeof window !== 'undefined') {
        window.location.href = '/';
      }
    }, 100);
    
  } catch (error) {
    console.warn('LocalStorage not available:', error);
    // Fallback: direct navigation
    if (typeof window !== 'undefined') {
      window.location.href = '/';
    }
  }
};

// Simple User Header Component with reactive updates
interface UserHeaderProps {
  showLogout?: boolean;
}

export const UserHeader: React.FC<UserHeaderProps> = ({ showLogout = true }) => {
  const navigate = useNavigate();
  const [currentUser, setCurrentUser] = useState<string>('Guest');
  const [isLoggingOut, setIsLoggingOut] = useState(false);

  // Update user name when component mounts and on storage changes
  useEffect(() => {
    const updateUserName = () => {
      const userName = getUserName();
      console.log('UserHeader: Updating user name to:', userName);
      setCurrentUser(userName);
    };

    // Initial load
    updateUserName();

    // Listen for storage changes (when user logs in/out in other tabs)
    const handleStorageChange = (e: StorageEvent) => {
      if (e.key === 'robotUser') {
        console.log('UserHeader: Storage changed, new value:', e.newValue);
        updateUserName();
      }
    };

    // Listen for custom events (when user logs in/out in same tab)
    const handleCustomUserChange = () => {
      console.log('UserHeader: Custom user change event received');
      updateUserName();
    };

    window.addEventListener('storage', handleStorageChange);
    window.addEventListener('userChanged', handleCustomUserChange);

    return () => {
      window.removeEventListener('storage', handleStorageChange);
      window.removeEventListener('userChanged', handleCustomUserChange);
    };
  }, []);

  const isGuest = currentUser === 'Guest';

  const handleLogout = async () => {
    if (isLoggingOut) return; // Prevent double-clicks
    
    console.log('UserHeader: Logout initiated');
    setIsLoggingOut(true);
    
    try {
      // FIXED: Clear user data properly
      logout();
      
      // FIXED: Also reset any face recognition state if possible
      try {
        // Try to reset face recognition state via API call
        const response = await fetch(`http://${window.location.hostname}:8000/api/reset-face-recognition`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
        });
        
        if (response.ok) {
          console.log('UserHeader: Face recognition state reset successfully');
        } else {
          console.warn('UserHeader: Failed to reset face recognition state');
        }
      } catch (error) {
        console.warn('UserHeader: Could not reset face recognition state:', error);
      }
      
    } catch (error) {
      console.error('UserHeader: Logout error:', error);
      setIsLoggingOut(false);
    }
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
    cursor: isLoggingOut ? 'not-allowed' : 'pointer',
    background: isLoggingOut 
      ? 'linear-gradient(135deg, #a0aec0, #718096)' 
      : 'linear-gradient(135deg, #f56565, #e53e3e)',
    color: 'white',
    fontSize: '0.9rem',
    fontWeight: '600' as const,
    transition: 'all 0.3s ease',
    opacity: isLoggingOut ? 0.6 : 1
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
            disabled={isLoggingOut}
            style={logoutButtonStyle}
            onMouseEnter={(e) => {
              if (!isLoggingOut) {
                e.currentTarget.style.transform = 'translateY(-2px)';
                e.currentTarget.style.boxShadow = '0 4px 8px rgba(245, 101, 101, 0.4)';
              }
            }}
            onMouseLeave={(e) => {
              if (!isLoggingOut) {
                e.currentTarget.style.transform = 'translateY(0)';
                e.currentTarget.style.boxShadow = 'none';
              }
            }}
            title={isLoggingOut ? "Logging out..." : "Return to face recognition"}
          >
            {isLoggingOut ? '🔄 Logging out...' : '🚪 Logout'}
          </button>
        )}
      </div>
    </div>
  );
};

// Enhanced setUserName that triggers updates
const setUserNameWithUpdate = (name: string): void => {
  setUserName(name);
  console.log('setUserNameWithUpdate: Set user to:', name);
};

// Export enhanced functions
export { getUserName, setUserNameWithUpdate as setUserName, logout };