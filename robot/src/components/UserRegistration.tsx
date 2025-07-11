import React, { useState, useEffect, useRef, useCallback } from 'react';
import { useNavigate } from 'react-router-dom';
import { setUserName } from './UserHeader';

interface EnhancedRobotStatus {
  status: string;
  current_user: string;
  camera_active: boolean;
  face_recognition_available: boolean;
  face_recognition_attempts: number;
  awaiting_registration: boolean;
}

type WSMessage =
  | { type: 'status_update'; data: EnhancedRobotStatus }
  | { type: 'user_registered'; data: { name: string; success: boolean } }
  | { type: 'face_recognition_reset'; data: { success: boolean } }
  | { type: 'error'; data: { message: string } }
  | { type: 'pong' };

const RegistrationPage: React.FC = () => {
  const navigate = useNavigate();
  
  // Core states
  const [response, setResponse] = useState<string>('');
  const [status, setStatus] = useState<EnhancedRobotStatus | null>(null);
  const [connectionStatus, setConnectionStatus] = useState<'connecting' | 'connected' | 'disconnected'>('connecting');
  const [retryCount, setRetryCount] = useState(0);
  const maxRetries = 5;

  // Registration states
  const [registrationName, setRegistrationName] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  // Use refs to avoid stale closure issues
  const wsRef = useRef<WebSocket | null>(null);
  const cameraRef = useRef<HTMLImageElement>(null);
  const mountedRef = useRef(true);
  const reconnectTimerRef = useRef<NodeJS.Timeout | null>(null);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      mountedRef.current = false;
      if (reconnectTimerRef.current) {
        clearTimeout(reconnectTimerRef.current);
      }
      if (wsRef.current) {
        wsRef.current.close();
      }
    };
  }, []);

  // Memoized message handler to avoid stale closures
  const handleWebSocketMessage = useCallback((event: MessageEvent) => {
    if (!mountedRef.current) return;
    
    try {
      const data: WSMessage = JSON.parse(event.data);
      console.log('📥 Registration received:', data.type, data);
      
      switch (data.type) {
        case 'status_update':
          setStatus(data.data);
          
          // If user was recognized during registration, go to menu
          if (data.data.current_user && 
              data.data.current_user !== 'Unknown' && 
              data.data.current_user !== '' && 
              !data.data.awaiting_registration) {
            console.log('✅ USER RECOGNIZED DURING REGISTRATION:', data.data.current_user);
            setUserName(data.data.current_user);
            setResponse(`Welcome back, ${data.data.current_user}!`);
            
            setTimeout(() => {
              navigate('/menu');
            }, 2000);
          }
          break;
          
        case 'user_registered':
          setIsLoading(false);
          if (data.data.success) {
            console.log('✅ REGISTRATION SUCCESS:', data.data.name);
            setUserName(data.data.name);
            setResponse(`Welcome, ${data.data.name}! Registration successful.`);
            
            setTimeout(() => {
              navigate('/menu');
            }, 2000);
          } else {
            setResponse('❌ Registration failed. Please try again.');
          }
          break;
          
        case 'face_recognition_reset':
          if (data.data.success) {
            setResponse('✅ Face recognition reset. You can try recognition again.');
          }
          break;
          
        case 'error':
          setResponse(`❌ Error: ${data.data.message}`);
          setIsLoading(false);
          break;
      }
    } catch (error) {
      console.error('Parse error:', error);
    }
  }, [navigate]);

  // Memoized connection handlers
  const handleWebSocketOpen = useCallback(() => {
    if (!mountedRef.current) return;
    console.log('✅ Registration WebSocket connected');
    setConnectionStatus('connected');
    setResponse('Connected - ready for registration');
    setRetryCount(0);
  }, []);

  const handleWebSocketError = useCallback(() => {
    console.error('Registration WebSocket error');
    setConnectionStatus('disconnected');
    setResponse('Connection error');
  }, []);

  const handleWebSocketClose = useCallback(() => {
    if (!mountedRef.current) return;
    
    console.log('Registration WebSocket closed');
    setConnectionStatus('disconnected');
    setResponse('Disconnected - reconnecting...');
    wsRef.current = null;
    
    reconnectTimerRef.current = setTimeout(() => {
      if (mountedRef.current) {
        setRetryCount(prev => prev + 1);
      }
    }, 3000);
  }, []);

  // WebSocket setup with proper event handler management
  useEffect(() => {
    if (retryCount >= maxRetries) {
      setConnectionStatus('disconnected');
      setResponse('Connection failed');
      return;
    }

    if (!mountedRef.current) return;
    
    setConnectionStatus('connecting');
    setResponse('Connecting to robot...');
    
    const websocket = new WebSocket(`ws://${window.location.hostname}:8000/ws`);
    wsRef.current = websocket;

    // Attach event listeners
    websocket.addEventListener('open', handleWebSocketOpen);
    websocket.addEventListener('message', handleWebSocketMessage);
    websocket.addEventListener('error', handleWebSocketError);
    websocket.addEventListener('close', handleWebSocketClose);

    return () => {
      if (reconnectTimerRef.current) {
        clearTimeout(reconnectTimerRef.current);
      }
      if (websocket) {
        websocket.removeEventListener('open', handleWebSocketOpen);
        websocket.removeEventListener('message', handleWebSocketMessage);
        websocket.removeEventListener('error', handleWebSocketError);
        websocket.removeEventListener('close', handleWebSocketClose);
        websocket.close();
      }
    };
  }, [retryCount, handleWebSocketOpen, handleWebSocketMessage, handleWebSocketError, handleWebSocketClose]);

  // Camera setup
  useEffect(() => {
    if (cameraRef.current && connectionStatus === 'connected') {
      const img = cameraRef.current;
      img.src = `http://${window.location.hostname}:8000/api/camera/stream`;
      
      img.onerror = () => {
        console.warn('Camera stream not available');
      };
      
      img.onload = () => {
        console.log('✅ Camera stream loaded successfully');
      };
    }
  }, [connectionStatus]);

  const sendWebSocketMessage = useCallback((type: string, data: any = {}) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      console.log('📤 Registration sending:', type, data);
      wsRef.current.send(JSON.stringify({ type, data }));
    } else {
      setResponse('Not connected');
    }
  }, []);

  const handleRegistration = useCallback(() => {
    if (registrationName.trim()) {
      setIsLoading(true);
      setResponse('📝 Registering your face...');
      sendWebSocketMessage('register_user', { name: registrationName.trim() });
    } else {
      setResponse('❌ Please enter your name');
    }
  }, [registrationName, sendWebSocketMessage]);

  const goBackToRecognition = useCallback(() => {
    console.log('🔄 GOING BACK TO FACE RECOGNITION');
    sendWebSocketMessage('reset_face_recognition');
    setTimeout(() => {
      navigate('/');
    }, 1000);
  }, [navigate, sendWebSocketMessage]);

  const skipToMenu = useCallback(() => {
    setUserName('Guest');
    navigate('/menu');
  }, [navigate]);

  return (
    <div style={styles.container}>
      <div style={styles.card}>
        <h1 style={styles.title}>📝 User Registration</h1>
        <p style={styles.subtitle}>Register your face for future recognition</p>
        
        <div style={styles.connectionIndicator}>
          <div style={{
            ...styles.statusDot,
            backgroundColor: connectionStatus === 'connected' ? '#48bb78' : 
                            connectionStatus === 'connecting' ? '#ed8936' : '#e53e3e'
          }}></div>
          <span>
            {connectionStatus === 'connected' ? 'Connected & Ready' : 
             connectionStatus === 'connecting' ? 'Connecting...' : 'Disconnected'}
          </span>
        </div>

        {connectionStatus === 'connected' && (
          <>
            <div style={styles.cameraContainer}>
              <img
                ref={cameraRef}
                alt="Robot Camera Feed"
                style={styles.cameraFeed}
                onError={() => console.warn('Camera image failed to load')}
                onLoad={() => console.log('Camera image loaded')}
              />
            </div>

            <div style={styles.message}>
              {isLoading && <span style={styles.loadingSpinner}></span>}
              {response}
            </div>

            <div style={styles.warningBox}>
              🆔 <strong>New User Registration</strong><br />
              The system doesn't recognize you. Please register to continue.
            </div>

            <div style={styles.inputGroup}>
              <input
                type="text"
                placeholder="Enter your full name"
                value={registrationName}
                onChange={(e) => setRegistrationName(e.target.value)}
                onKeyPress={(e) => e.key === 'Enter' && handleRegistration()}
                style={styles.input}
                autoFocus
              />
              <button
                onClick={handleRegistration}
                disabled={!registrationName.trim() || isLoading}
                style={{
                  ...styles.primaryBtn,
                  opacity: (!registrationName.trim() || isLoading) ? 0.6 : 1,
                  cursor: (!registrationName.trim() || isLoading) ? 'not-allowed' : 'pointer'
                }}
              >
                {isLoading ? '📝 Registering...' : '📝 Register My Face'}
              </button>
            </div>

            <div style={styles.tipBox}>
              💡 <strong>Registration Tips:</strong><br />
              • Look directly at the camera<br />
              • Ensure good lighting<br />
              • Keep your face visible and centered
            </div>
            
            <div style={styles.buttonRow}>
              <button 
                onClick={goBackToRecognition} 
                style={styles.secondaryBtn}
                disabled={isLoading}
              >
                🔄 Try Face Recognition Again
              </button>
              <button 
                onClick={skipToMenu} 
                style={styles.skipBtn}
                disabled={isLoading}
              >
                👤 Skip & Continue as Guest
              </button>
            </div>

            {/* Debug Info */}
            {status && (
              <div style={styles.debugBox}>
                <strong>Registration Debug:</strong><br />
                WS Status: {connectionStatus}<br />
                Backend User: {status.current_user}<br />
                Backend Attempts: {status.face_recognition_attempts}/3<br />
                Awaiting Registration: {status.awaiting_registration ? 'Yes' : 'No'}<br />
                Camera: {status.camera_active ? 'Active' : 'Inactive'}<br />
                Response: {response}
              </div>
            )}
          </>
        )}

        {connectionStatus !== 'connected' && (
          <div>
            <div style={styles.disconnectedMessage}>
              {connectionStatus === 'connecting' ? 'Connecting to robot...' : 'Connection failed'}
            </div>
            
            {connectionStatus === 'disconnected' && (
              <div style={styles.buttonRow}>
                <button onClick={() => setRetryCount(0)} style={styles.refreshBtn}>
                  🔄 Retry Connection
                </button>
                <button onClick={skipToMenu} style={styles.skipBtn}>
                  👤 Continue as Guest
                </button>
              </div>
            )}
          </div>
        )}
      </div>
    </div>
  );
};

const styles = {
  container: {
    minHeight: '100vh',
    width: '100vw',
    background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
    display: 'flex' as const,
    justifyContent: 'center',
    alignItems: 'center',
    fontFamily: '"Inter", sans-serif',
    padding: '2rem'
  },
  card: {
    background: 'rgba(255, 255, 255, 0.95)',
    backdropFilter: 'blur(20px)',
    borderRadius: '24px',
    padding: '3rem',
    boxShadow: '0 20px 40px rgba(0, 0, 0, 0.1)',
    border: '1px solid rgba(255, 255, 255, 0.2)',
    textAlign: 'center' as const,
    width: '100%',
    maxWidth: '600px'
  },
  title: {
    fontSize: '2.5rem',
    fontWeight: '700',
    color: '#2d3748',
    marginBottom: '1rem',
    background: 'linear-gradient(135deg, #667eea, #764ba2)',
    WebkitBackgroundClip: 'text',
    WebkitTextFillColor: 'transparent',
    backgroundClip: 'text'
  },
  subtitle: {
    color: '#718096',
    marginBottom: '2rem',
    fontSize: '1.2rem'
  },
  connectionIndicator: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    marginBottom: '2rem',
    fontSize: '1rem'
  },
  statusDot: {
    width: '12px',
    height: '12px',
    borderRadius: '50%',
    marginRight: '10px'
  },
  cameraContainer: {
    textAlign: 'center' as const,
    marginBottom: '2rem'
  },
  cameraFeed: {
    width: '100%',
    maxWidth: '400px',
    height: 'auto',
    borderRadius: '16px',
    border: '3px solid #e2e8f0',
    boxShadow: '0 8px 16px rgba(0, 0, 0, 0.1)',
    minHeight: '300px',
    backgroundColor: '#f7fafc'
  },
  message: {
    fontSize: '1.2rem',
    fontWeight: '600',
    color: '#2d3748',
    marginBottom: '2rem',
    minHeight: '2rem',
    padding: '1rem',
    background: 'rgba(247, 250, 252, 0.8)',
    borderRadius: '12px'
  },
  warningBox: {
    fontSize: '1.1rem',
    color: '#4a5568',
    marginBottom: '2rem',
    padding: '1rem',
    background: 'rgba(254, 215, 170, 0.8)',
    borderRadius: '12px',
    border: '1px solid rgba(251, 146, 60, 0.4)'
  },
  tipBox: {
    fontSize: '0.9rem',
    color: '#4a5568',
    marginBottom: '2rem',
    padding: '1rem',
    background: 'rgba(219, 234, 254, 0.8)',
    borderRadius: '12px',
    border: '1px solid rgba(59, 130, 246, 0.3)',
    textAlign: 'left' as const
  },
  inputGroup: {
    display: 'flex',
    gap: '1rem',
    marginBottom: '2rem',
    alignItems: 'center'
  },
  input: {
    flex: 1,
    padding: '1rem',
    borderRadius: '12px',
    border: '2px solid #e2e8f0',
    fontSize: '1rem'
  },
  buttonRow: {
    display: 'flex',
    gap: '1rem',
    justifyContent: 'center',
    flexWrap: 'wrap' as const,
    marginBottom: '1rem'
  },
  primaryBtn: {
    padding: '1rem 2rem',
    borderRadius: '12px',
    border: 'none',
    cursor: 'pointer',
    background: 'linear-gradient(135deg, #667eea, #764ba2)',
    color: 'white',
    fontSize: '1rem',
    fontWeight: '600'
  },
  secondaryBtn: {
    padding: '0.75rem 1.5rem',
    borderRadius: '8px',
    border: 'none',
    cursor: 'pointer',
    background: 'linear-gradient(135deg, #48bb78, #38a169)',
    color: 'white',
    fontSize: '0.9rem',
    fontWeight: '600'
  },
  skipBtn: {
    padding: '0.75rem 1.5rem',
    borderRadius: '8px',
    border: '2px solid #e2e8f0',
    cursor: 'pointer',
    background: 'transparent',
    color: '#718096',
    fontSize: '0.9rem',
    fontWeight: '600'
  },
  refreshBtn: {
    padding: '0.75rem 1.5rem',
    borderRadius: '8px',
    border: 'none',
    cursor: 'pointer',
    background: 'linear-gradient(135deg, #ed8936, #dd6b20)',
    color: 'white',
    fontSize: '0.9rem',
    fontWeight: '600'
  },
  loadingSpinner: {
    display: 'inline-block',
    width: '20px',
    height: '20px',
    border: '2px solid #f3f3f3',
    borderTop: '2px solid #667eea',
    borderRadius: '50%',
    animation: 'spin 1s linear infinite',
    marginRight: '10px'
  },
  debugBox: {
    marginTop: '2rem',
    padding: '1rem',
    background: 'rgba(240, 244, 248, 0.8)',
    borderRadius: '8px',
    fontSize: '0.8rem',
    color: '#4a5568',
    textAlign: 'left' as const
  },
  disconnectedMessage: {
    fontSize: '1.1rem',
    color: '#718096',
    marginBottom: '2rem'
  }
};

// Add CSS animation for spinner
const spinKeyframes = `
  @keyframes spin {
    0% { transform: rotate(0deg); }
    100% { transform: rotate(360deg); }
  }
`;

// Inject CSS
if (typeof document !== 'undefined') {
  const style = document.createElement('style');
  style.textContent = spinKeyframes;
  document.head.appendChild(style);
}

export default RegistrationPage;