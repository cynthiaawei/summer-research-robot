import React, { useState, useEffect, useRef } from 'react';
import { useNavigate } from 'react-router-dom';
import { useUser } from './UserContext';

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
  | { type: 'user_recognized'; data: { user: string | null; success: boolean } }
  | { type: 'error'; data: { message: string } }
  | { type: 'ping' }
  | { type: 'pong' };

const FaceRecognitionGate: React.FC = () => {
  const { setCurrentUser } = useUser();
  const navigate = useNavigate();
  
  // Core states - SAME AS WORKING COMPONENTS
  const [ws, setWs] = useState<WebSocket | null>(null);
  const [response, setResponse] = useState<string>('');
  const [status, setStatus] = useState<EnhancedRobotStatus | null>(null);
  const [connectionStatus, setConnectionStatus] = useState<'connecting' | 'connected' | 'disconnected'>('connecting');
  const [retryCount, setRetryCount] = useState(0);
  const maxRetries = 5;
  const [isNavigating, setIsNavigating] = useState(false);

  const cameraRef = useRef<HTMLImageElement>(null);
  const mountedRef = useRef(true);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      mountedRef.current = false;
    };
  }, []);

  // WebSocket setup - EXACT SAME AS arrow_keys.tsx
  useEffect(() => {
    let reconnectTimer: number;
    let pingInterval: number;
    
    const connect = () => {
      if (!mountedRef.current) return;
      
      setConnectionStatus('connecting');
      const websocket = new WebSocket(`ws://${window.location.hostname}:8000/ws`);
      setWs(websocket);

      websocket.onopen = () => {
        if (!mountedRef.current) return;
        console.log('✅ Face Recognition WebSocket connected');
        setConnectionStatus('connected');
        setResponse('🔍 Scanning for faces...');
        setRetryCount(0);

        // Set up ping interval to keep connection alive
        pingInterval = window.setInterval(() => {
          if (websocket.readyState === WebSocket.OPEN) {
            websocket.send(JSON.stringify({ type: 'ping' }));
          }
        }, 25000);
      };

      websocket.onmessage = (event) => {
        if (!mountedRef.current) return;
        
        try {
          const data: WSMessage = JSON.parse(event.data);
          console.log('📥 Face Recognition received:', data.type, data);
          
          switch (data.type) {
            case 'status_update':
              setStatus(data.data);
              
              console.log('🔍 Face Recognition Status:', {
                awaiting_registration: data.data.awaiting_registration,
                current_user: data.data.current_user,
                attempts: data.data.face_recognition_attempts
              });
              
              // If user recognized - go to menu
              if (data.data.current_user && 
                  data.data.current_user !== 'Unknown' && 
                  data.data.current_user !== '' && 
                  !data.data.awaiting_registration &&
                  !isNavigating) {
                console.log('✅ USER RECOGNIZED:', data.data.current_user);
                setIsNavigating(true);
                setCurrentUser(data.data.current_user);
                setResponse(`🎉 Welcome back, ${data.data.current_user}!`);
                
                setTimeout(() => {
                  if (mountedRef.current) {
                    navigate('/menu');
                  }
                }, 2000);
              }
              // If awaiting registration - go to registration page
              else if (data.data.awaiting_registration && !isNavigating) {
                console.log('🔄 REDIRECTING TO REGISTRATION');
                setIsNavigating(true);
                setResponse('❓ User not recognized. Redirecting to registration...');
                
                setTimeout(() => {
                  if (mountedRef.current) {
                    navigate('/register');
                  }
                }, 1500);
              }
              // Show scanning progress
              else if (data.data.face_recognition_attempts > 0 && 
                       data.data.face_recognition_attempts < 3 &&
                       !data.data.awaiting_registration) {
                setResponse(`🔍 Scanning for faces... (${data.data.face_recognition_attempts}/3)`);
              }
              else if (!data.data.awaiting_registration && data.data.current_user === 'Unknown') {
                setResponse('🔍 Scanning for faces...');
              }
              break;
              
            case 'user_recognized':
              if (data.data.success && data.data.user && !isNavigating) {
                console.log('✅ USER RECOGNIZED VIA MESSAGE:', data.data.user);
                setIsNavigating(true);
                setCurrentUser(data.data.user);
                setResponse(`🎉 Welcome back, ${data.data.user}!`);
                
                setTimeout(() => {
                  if (mountedRef.current) {
                    navigate('/menu');
                  }
                }, 2000);
              }
              break;
              
            case 'face_recognition_reset':
              if (data.data.success) {
                setResponse('🔄 Face recognition reset. Starting new scan...');
                setIsNavigating(false);
              }
              break;
              
            case 'error':
              setResponse(`❌ Error: ${data.data.message}`);
              break;

            case 'pong':
              // Connection is alive
              break;
          }
        } catch (error) {
          console.error('Face Recognition parse error:', error);
        }
      };

      websocket.onerror = () => {
        console.error('Face Recognition WebSocket error');
        setConnectionStatus('disconnected');
        setResponse('❌ Connection error');
      };

      websocket.onclose = () => {
        if (!mountedRef.current) return;
        
        console.log('Face Recognition WebSocket closed');
        setConnectionStatus('disconnected');
        setResponse('🔄 Disconnected - reconnecting...');
        setWs(null);
        
        if (pingInterval) {
          clearInterval(pingInterval);
        }
        
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
      setResponse('❌ Connection failed after maximum retries');
    }

    return () => {
      if (reconnectTimer) {
        clearTimeout(reconnectTimer);
      }
      if (pingInterval) {
        clearInterval(pingInterval);
      }
      if (ws) {
        ws.close();
      }
    };
  }, [retryCount, navigate, setCurrentUser, isNavigating]);

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
      console.log('📤 Face Recognition sending:', type, data);
      ws.send(JSON.stringify({ type, data }));
    } else {
      setResponse('❌ Not connected');
    }
  };

  const resetRecognition = () => {
    console.log('🔄 RESETTING FACE RECOGNITION');
    setResponse('🔄 Resetting face recognition...');
    setIsNavigating(false);
    sendWebSocketMessage('reset_face_recognition');
  };

  const goToMenu = () => {
    setCurrentUser('Guest');
    navigate('/menu');
  };

  return (
    <div style={styles.container}>
      <div style={styles.card}>
        <div style={styles.header}>
          <h1 style={styles.title}>🤖 Robot Access Control</h1>
          <p style={styles.subtitle}>Advanced face recognition security system</p>
        </div>
        
        <div style={styles.connectionIndicator}>
          <div style={{
            ...styles.statusDot,
            backgroundColor: connectionStatus === 'connected' ? '#48bb78' : 
                            connectionStatus === 'connecting' ? '#ed8936' : '#e53e3e'
          }}></div>
          <span>
            {connectionStatus === 'connected' ? '🟢 Connected & Scanning' : 
             connectionStatus === 'connecting' ? '🟡 Connecting...' : '🔴 Disconnected'}
          </span>
        </div>

        {connectionStatus === 'connected' && (
          <>
            <div style={styles.cameraContainer}>
              <img
                ref={cameraRef}
                alt="Robot Camera Feed"
                style={styles.cameraFeed}
              />
            </div>

            <div style={styles.message}>
              {isNavigating && <span style={styles.loadingSpinner}></span>}
              {response}
            </div>

            <div style={styles.infoBox}>
              👁️ <strong>Face Recognition Active</strong><br />
              Please look directly at the camera for identification
            </div>
            
            <div style={styles.buttonRow}>
              <button 
                onClick={resetRecognition} 
                style={styles.secondaryBtn}
                disabled={isNavigating}
              >
                🔄 Reset & Try Again
              </button>
              
              <button 
                onClick={goToMenu} 
                style={styles.guestBtn}
                disabled={isNavigating}
              >
                👤 Continue as Guest
              </button>
            </div>

            {/* Progress Indicator */}
            {status && (
              <div style={styles.progressBox}>
                <div style={styles.progressHeader}>
                  <strong>Recognition Progress:</strong>
                </div>
                <div style={styles.progressBar}>
                  <div 
                    style={{
                      ...styles.progressFill,
                      width: `${(status.face_recognition_attempts / 3) * 100}%`,
                      backgroundColor: status.face_recognition_attempts >= 3 ? '#e53e3e' : '#3b82f6'
                    }}
                  ></div>
                </div>
                <div style={styles.progressText}>
                  Attempts: {status.face_recognition_attempts}/3
                  {status.awaiting_registration && ' - Registration Required'}
                </div>
              </div>
            )}

            {/* Debug Info */}
            {status && (
              <div style={styles.debugBox}>
                <strong>🔧 Debug Info:</strong><br />
                Backend User: {status.current_user}<br />
                Backend Attempts: {status.face_recognition_attempts}/3<br />
                Awaiting Registration: {status.awaiting_registration ? 'Yes' : 'No'}<br />
                Camera: {status.camera_active ? 'Active' : 'Inactive'}<br />
                Face Recognition: {status.face_recognition_available ? 'Available' : 'Unavailable'}
              </div>
            )}
          </>
        )}

        {connectionStatus !== 'connected' && (
          <div style={styles.disconnectedContainer}>
            <div style={styles.disconnectedMessage}>
              {connectionStatus === 'connecting' ? 
                '🔄 Connecting to robot system...' : 
                '❌ Connection failed'}
              {retryCount > 0 && ` (Attempt ${retryCount}/${maxRetries})`}
            </div>
            
            {connectionStatus === 'disconnected' && retryCount < maxRetries && (
              <div style={styles.retryInfo}>
                Retrying automatically in 3 seconds...
              </div>
            )}
            
            {retryCount >= maxRetries && (
              <div style={styles.buttonRow}>
                <button onClick={() => setRetryCount(0)} style={styles.refreshBtn}>
                  🔄 Retry Connection
                </button>
                <button onClick={goToMenu} style={styles.guestBtn}>
                  👤 Continue as Guest
                </button>
              </div>
            )}
          </div>
        )}

        <div style={styles.instructionsBox}>
          <h4 style={styles.instructionsTitle}>📋 How it works:</h4>
          <ul style={styles.instructionsList}>
            <li><strong>Face Recognition:</strong> Look directly at the camera for identification</li>
            <li><strong>First Time Users:</strong> After 3 failed attempts, you'll be asked to register</li>
            <li><strong>Guest Access:</strong> Use "Continue as Guest" to skip face recognition</li>
            <li><strong>Troubleshooting:</strong> Use "Reset & Try Again" if recognition fails</li>
          </ul>
        </div>
      </div>

      {/* Loading Animation Styles */}
      <style>{`
        @keyframes spin {
          0% { transform: rotate(0deg); }
          100% { transform: rotate(360deg); }
        }
        
        @keyframes pulse {
          0%, 100% { opacity: 1; }
          50% { opacity: 0.5; }
        }
      `}</style>
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
    maxWidth: '700px',
    position: 'relative' as const
  },
  header: {
    marginBottom: '2rem'
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
    fontSize: '1.2rem',
    margin: 0
  },
  connectionIndicator: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    marginBottom: '2rem',
    fontSize: '1rem',
    fontWeight: '600' as const,
    padding: '0.75rem',
    background: 'rgba(247, 250, 252, 0.8)',
    borderRadius: '12px',
    border: '1px solid rgba(226, 232, 240, 0.5)'
  },
  statusDot: {
    width: '12px',
    height: '12px',
    borderRadius: '50%',
    marginRight: '10px',
    animation: 'pulse 2s infinite'
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
    borderRadius: '12px',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    gap: '0.5rem'
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
  progressBox: {
    marginBottom: '2rem',
    padding: '1rem',
    background: 'rgba(240, 244, 248, 0.8)',
    borderRadius: '12px',
    border: '1px solid rgba(226, 232, 240, 0.5)'
  },
  progressHeader: {
    fontSize: '0.9rem',
    fontWeight: '600' as const,
    color: '#4a5568',
    marginBottom: '0.5rem'
  },
  progressBar: {
    width: '100%',
    height: '8px',
    background: '#e2e8f0',
    borderRadius: '4px',
    overflow: 'hidden',
    marginBottom: '0.5rem'
  },
  progressFill: {
    height: '100%',
    transition: 'width 0.3s ease',
    borderRadius: '4px'
  },
  progressText: {
    fontSize: '0.8rem',
    color: '#718096'
  },
  buttonRow: {
    display: 'flex',
    gap: '1rem',
    justifyContent: 'center',
    flexWrap: 'wrap' as const,
    marginBottom: '2rem'
  },
  secondaryBtn: {
    padding: '0.75rem 1.5rem',
    borderRadius: '8px',
    border: 'none',
    cursor: 'pointer',
    background: 'linear-gradient(135deg, #48bb78, #38a169)',
    color: 'white',
    fontSize: '0.9rem',
    fontWeight: '600',
    transition: 'all 0.3s ease'
  },
  guestBtn: {
    padding: '0.75rem 1.5rem',
    borderRadius: '8px',
    border: '2px solid #667eea',
    cursor: 'pointer',
    background: 'transparent',
    color: '#667eea',
    fontSize: '0.9rem',
    fontWeight: '600',
    transition: 'all 0.3s ease'
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
    animation: 'spin 1s linear infinite'
  },
  disconnectedContainer: {
    padding: '2rem',
    textAlign: 'center' as const
  },
  disconnectedMessage: {
    fontSize: '1.1rem',
    color: '#4a5568',
    marginBottom: '1rem'
  },
  retryInfo: {
    fontSize: '0.9rem',
    color: '#718096',
    marginBottom: '2rem',
    fontStyle: 'italic'
  },
  debugBox: {
    marginBottom: '2rem',
    padding: '1rem',
    background: 'rgba(240, 244, 248, 0.8)',
    borderRadius: '8px',
    fontSize: '0.8rem',
    color: '#4a5568',
    textAlign: 'left' as const
  },
  instructionsBox: {
    padding: '1rem',
    background: 'rgba(240, 244, 248, 0.8)',
    borderRadius: '12px',
    textAlign: 'left' as const,
    fontSize: '0.9rem',
    color: '#4a5568'
  },
  instructionsTitle: {
    marginTop: 0,
    marginBottom: '0.5rem',
    fontSize: '1rem',
    color: '#2d3748'
  },
  instructionsList: {
    margin: 0,
    paddingLeft: '1.5rem',
    lineHeight: '1.6'
  }
};

export default FaceRecognitionGate;