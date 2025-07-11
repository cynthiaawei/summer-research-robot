import React, { useState, useEffect, useRef } from 'react';
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
  | { type: 'face_recognition_reset'; data: { success: boolean } }
  | { type: 'error'; data: { message: string } }
  | { type: 'pong' };

const FaceRecognitionGate: React.FC = () => {
  const navigate = useNavigate();
  
  // Core states
  const [ws, setWs] = useState<WebSocket | null>(null);
  const [response, setResponse] = useState<string>('');
  const [status, setStatus] = useState<EnhancedRobotStatus | null>(null);
  const [connectionStatus, setConnectionStatus] = useState<'connecting' | 'connected' | 'disconnected'>('connecting');
  const [retryCount, setRetryCount] = useState(0);
  const maxRetries = 5;

  const cameraRef = useRef<HTMLImageElement>(null);
  const mountedRef = useRef(true);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      mountedRef.current = false;
    };
  }, []);

  // WebSocket setup
  useEffect(() => {
    let reconnectTimer: number;
    
    const connect = () => {
      if (!mountedRef.current) return;
      
      setConnectionStatus('connecting');
      const websocket = new WebSocket(`ws://${window.location.hostname}:8000/ws`);
      setWs(websocket);

      websocket.onopen = () => {
        if (!mountedRef.current) return;
        console.log('✅ WebSocket connected');
        setConnectionStatus('connected');
        setResponse('Scanning for faces...');
        setRetryCount(0);
      };

      websocket.onmessage = (event) => {
        if (!mountedRef.current) return;
        
        try {
          const data: WSMessage = JSON.parse(event.data);
          console.log('📥 Received:', data.type, data);
          
          switch (data.type) {
            case 'status_update':
              setStatus(data.data);
              
              // If user recognized - go to menu
              if (data.data.current_user && 
                  data.data.current_user !== 'Unknown' && 
                  data.data.current_user !== '' && 
                  !data.data.awaiting_registration) {
                console.log('✅ USER RECOGNIZED:', data.data.current_user);
                setUserName(data.data.current_user);
                setResponse(`Welcome back, ${data.data.current_user}!`);
                
                setTimeout(() => {
                  navigate('/menu');
                }, 2000);
              }
              // If awaiting registration - go to registration page
              else if (data.data.awaiting_registration) {
                console.log('🔄 REDIRECTING TO REGISTRATION');
                navigate('/register');
              }
              // Show scanning progress
              else if (data.data.face_recognition_attempts > 0 && 
                       data.data.face_recognition_attempts < 3) {
                setResponse(`Scanning for faces... (${data.data.face_recognition_attempts}/3)`);
              }
              else {
                setResponse('Scanning for faces...');
              }
              break;
              
            case 'face_recognition_reset':
              if (data.data.success) {
                setResponse('Face recognition reset. Scanning...');
              }
              break;
              
            case 'error':
              setResponse(`Error: ${data.data.message}`);
              break;
          }
        } catch (error) {
          console.error('Parse error:', error);
        }
      };

      websocket.onerror = () => {
        console.error('WebSocket error');
        setConnectionStatus('disconnected');
        setResponse('Connection error');
      };

      websocket.onclose = () => {
        if (!mountedRef.current) return;
        
        console.log('WebSocket closed');
        setConnectionStatus('disconnected');
        setResponse('Disconnected - reconnecting...');
        setWs(null);
        
        reconnectTimer = window.setTimeout(() => {
          if (mountedRef.current) {
            setRetryCount(prev => prev + 1);
          }
        }, 3000);
      };
    };

    if (retryCount < maxRetries) {
      connect();
    } else {
      setConnectionStatus('disconnected');
      setResponse('Connection failed after maximum retries');
    }

    return () => {
      if (reconnectTimer) {
        clearTimeout(reconnectTimer);
      }
      if (ws) {
        ws.close();
      }
    };
  }, [retryCount, navigate]);

  // Camera setup
  useEffect(() => {
    if (cameraRef.current && connectionStatus === 'connected') {
      const img = cameraRef.current;
      img.src = `http://${window.location.hostname}:8000/api/camera/stream`;
      
      img.onerror = () => {
        console.warn('Camera stream not available');
      };
    }
  }, [connectionStatus]);

  const sendWebSocketMessage = (type: string, data: any = {}) => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      console.log('📤 Sending:', type, data);
      ws.send(JSON.stringify({ type, data }));
    } else {
      setResponse('Not connected');
    }
  };

  const resetRecognition = () => {
    console.log('🔄 RESETTING FACE RECOGNITION');
    setResponse('Resetting face recognition...');
    sendWebSocketMessage('reset_face_recognition');
  };

  const continueAsGuest = () => {
    console.log('👤 CONTINUING AS GUEST');
    setUserName('Guest');
    navigate('/menu');
  };

  const goToRegistration = () => {
    console.log('📝 GOING TO REGISTRATION');
    navigate('/register');
  };

  return (
    <div style={styles.container}>
      <div style={styles.card}>
        <h1 style={styles.title}>🤖 Robot Access Control</h1>
        <p style={styles.subtitle}>Face recognition security system</p>
        
        <div style={styles.connectionIndicator}>
          <div style={{
            ...styles.statusDot,
            backgroundColor: connectionStatus === 'connected' ? '#48bb78' : 
                            connectionStatus === 'connecting' ? '#ed8936' : '#e53e3e'
          }}></div>
          <span>
            {connectionStatus === 'connected' ? 'Connected' : 
             connectionStatus === 'connecting' ? 'Connecting...' : 'Disconnected'}
          </span>
        </div>
        
        <div style={styles.cameraContainer}>
          <img
            ref={cameraRef}
            alt="Robot Camera Feed"
            style={styles.cameraFeed}
          />
        </div>

        <div style={styles.message}>
          {response}
        </div>

        <div style={styles.infoBox}>
          👁️ <strong>Face Recognition Active</strong><br />
          Please look at the camera for identification
        </div>
        
        <div style={styles.buttonRow}>
          <button onClick={resetRecognition} style={styles.secondaryBtn}>
            🔄 Reset & Try Again
          </button>
          <button onClick={continueAsGuest} style={styles.guestBtn}>
            👤 Continue as Guest
          </button>
          <button onClick={goToRegistration} style={styles.registerBtn}>
            📝 Register as User
          </button>
        </div>

        {/* Debug Info */}
        {status && (
          <div style={styles.debugBox}>
            <strong>Debug Info:</strong><br />
            Backend User: {status.current_user}<br />
            Backend Attempts: {status.face_recognition_attempts}/3<br />
            Awaiting Registration: {status.awaiting_registration ? 'Yes' : 'No'}<br />
            Camera: {status.camera_active ? 'Active' : 'Inactive'}
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
    marginBottom: '1rem',
    fontSize: '0.9rem'
  },
  statusDot: {
    width: '8px',
    height: '8px',
    borderRadius: '50%',
    marginRight: '8px'
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
    boxShadow: '0 8px 16px rgba(0, 0, 0, 0.1)'
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
  infoBox: {
    fontSize: '1.1rem',
    color: '#4a5568',
    marginBottom: '2rem',
    padding: '1rem',
    background: 'rgba(219, 234, 254, 0.8)',
    borderRadius: '12px',
    border: '1px solid rgba(59, 130, 246, 0.3)'
  },
  buttonRow: {
    display: 'flex',
    gap: '0.5rem',
    justifyContent: 'center',
    flexWrap: 'wrap' as const,
    marginBottom: '1rem'
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
  guestBtn: {
    padding: '0.75rem 1.5rem',
    borderRadius: '8px',
    border: '2px solid #667eea',
    cursor: 'pointer',
    background: 'transparent',
    color: '#667eea',
    fontSize: '0.9rem',
    fontWeight: '600'
  },
  registerBtn: {
    padding: '0.75rem 1.5rem',
    borderRadius: '8px',
    border: 'none',
    cursor: 'pointer',
    background: 'linear-gradient(135deg, #667eea, #764ba2)',
    color: 'white',
    fontSize: '0.9rem',
    fontWeight: '600'
  },
  debugBox: {
    marginTop: '2rem',
    padding: '1rem',
    background: 'rgba(240, 244, 248, 0.8)',
    borderRadius: '8px',
    fontSize: '0.8rem',
    color: '#4a5568',
    textAlign: 'left' as const
  }
};

export default FaceRecognitionGate;
