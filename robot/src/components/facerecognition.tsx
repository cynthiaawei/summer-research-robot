import React, { useState, useEffect, useRef } from 'react';
import { useUser } from './UserContext';

interface EnhancedRobotStatus {
  status: string;
  message: string;
  obstacle_detected: boolean;
  obstacle_sensor?: string;
  obstacle_distance?: number;
  current_speeds: Record<string, number>;
  sensor_distances?: Record<string, number>;
  last_command: string;
  uptime: number;
  gpio_available?: boolean;
  // Enhanced fields
  current_user: string;
  faces_detected: string[];
  hand_gesture: string;
  camera_active: boolean;
  last_speech_output: string;
  listening: boolean;
  speech_recognition_active: boolean;
  interaction_mode: string;
  face_recognition_available: boolean;
  speech_recognition_available: boolean;
  mediapipe_available: boolean;
  face_recognition_attempts: number;
  awaiting_registration: boolean;
}

type WSMessage =
  | { type: 'status_update'; data: EnhancedRobotStatus }
  | { type: 'user_recognized'; data: { user: string | null; success: boolean } }
  | { type: 'user_registered'; data: { name: string; success: boolean } }
  | { type: 'speech_output'; data: { text: string; success: boolean } }
  | { type: 'face_recognition_reset'; data: { success: boolean } }
  | { type: 'error'; data: { message: string } }
  | { type: 'ping' }
  | { type: 'pong' };

const FaceRecognitionGate: React.FC = () => {
  const { setCurrentUser } = useUser();
  
  // States - using same pattern as working components
  const [ws, setWs] = useState<WebSocket | null>(null);
  const [response, setResponse] = useState<string>('');
  const [status, setStatus] = useState<EnhancedRobotStatus | null>(null);
  const [connectionStatus, setConnectionStatus] = useState<'connecting' | 'connected' | 'disconnected'>('connecting');
  const [retryCount, setRetryCount] = useState(0);
  const maxRetries = 5;

  // Face recognition specific states
  const [recognitionStage, setRecognitionStage] = useState<'scanning' | 'unknown' | 'recognized'>('scanning');
  const [userName, setUserName] = useState('');
  const [registrationName, setRegistrationName] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const cameraRef = useRef<HTMLImageElement>(null);
  const mountedRef = useRef(true);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      mountedRef.current = false;
    };
  }, []);

  // WebSocket connection - EXACT SAME PATTERN as working components
  useEffect(() => {
    let reconnectTimer: number;
    
    const connect = () => {
      if (!mountedRef.current) return;
      
      setConnectionStatus('connecting');
      const websocket = new WebSocket(`ws://${window.location.hostname}:8000/ws`);
      setWs(websocket);

      websocket.onopen = () => {
        if (!mountedRef.current) return;
        console.log('WebSocket connected');
        setConnectionStatus('connected');
        setResponse('Connected to enhanced robot');
        setRetryCount(0);
      };

      websocket.onmessage = (event) => {
        if (!mountedRef.current) return;
        
        try {
          const data: WSMessage = JSON.parse(event.data);
          
          switch (data.type) {
            case 'status_update':
              setStatus(data.data);
              
              // Handle face recognition state transitions
              if (data.data.awaiting_registration && recognitionStage !== 'unknown') {
                console.log('Backend says awaiting registration - switching to unknown stage');
                setRecognitionStage('unknown');
                setResponse('I don\'t recognize you. Please enter your name to register.');
              } else if (data.data.current_user && 
                        data.data.current_user !== 'Unknown' && 
                        data.data.current_user !== '' && 
                        !data.data.awaiting_registration &&
                        recognitionStage !== 'recognized') {
                console.log(`Backend recognized user: ${data.data.current_user}`);
                setUserName(data.data.current_user);
                setRecognitionStage('recognized');
                setResponse(`Welcome back, ${data.data.current_user}!`);
                setCurrentUser(data.data.current_user);
                
                setTimeout(() => {
                  if (mountedRef.current) {
                    window.location.href = '/menu';
                  }
                }, 3000);
              } else if (data.data.face_recognition_attempts > 0 && 
                        data.data.face_recognition_attempts < 3 && 
                        recognitionStage === 'scanning' &&
                        !data.data.awaiting_registration) {
                setResponse(`Scanning for faces... (${data.data.face_recognition_attempts}/3)`);
              }
              break;
              
            case 'user_registered':
              setIsLoading(false);
              if (data.data.success) {
                setUserName(data.data.name);
                setRecognitionStage('recognized');
                setResponse(`Nice to meet you, ${data.data.name}! Registration successful.`);
                setCurrentUser(data.data.name);
                
                setTimeout(() => {
                  if (mountedRef.current) {
                    window.location.href = '/menu';
                  }
                }, 3000);
              } else {
                setResponse('Registration failed. Please try again.');
                setRecognitionStage('unknown');
              }
              break;
              
            case 'face_recognition_reset':
              if (data.data.success) {
                setRecognitionStage('scanning');
                setResponse('Scanning for faces...');
              }
              break;
              
            case 'error':
              setResponse(`Error: ${data.data.message}`);
              setIsLoading(false);
              break;
          }
        } catch (error) {
          console.error('WebSocket message parse error:', error);
        }
      };

      websocket.onerror = () => {
        console.error('WebSocket error');
        setConnectionStatus('disconnected');
        setResponse('WebSocket connection error');
      };

      websocket.onclose = () => {
        if (!mountedRef.current) return;
        
        console.log('WebSocket closed');
        setConnectionStatus('disconnected');
        setResponse('WebSocket disconnected - reconnecting...');
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
      setResponse('Max retries reached');
    }

    return () => {
      if (reconnectTimer) {
        clearTimeout(reconnectTimer);
      }
      if (ws) {
        ws.close();
      }
    };
  }, [retryCount]); // Same dependency as working components

  // Camera stream setup
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
      ws.send(JSON.stringify({ type, data }));
    } else {
      setResponse('WebSocket not connected');
    }
  };

  const handleTextRegistration = () => {
    if (registrationName.trim()) {
      setIsLoading(true);
      setResponse('Registering your face...');
      sendWebSocketMessage('register_user', { name: registrationName.trim() });
    }
  };

  const skipToMenu = () => {
    setCurrentUser('Guest');
    window.location.href = '/menu';
  };

  const proceedToControls = () => {
    setCurrentUser(userName);
    window.location.href = '/menu';
  };

  const retryRecognition = () => {
    setRecognitionStage('scanning');
    setResponse('Resetting face recognition...');
    sendWebSocketMessage('reset_face_recognition');
  };

  const refreshConnection = () => {
    if (ws) {
      ws.close();
    }
  };

  return (
    <div style={styles.container}>
      <style>{`
        @keyframes spin {
          0% { transform: rotate(0deg); }
          100% { transform: rotate(360deg); }
        }
      `}</style>
      
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
              {isLoading && <span style={styles.loadingSpinner}></span>}
              {response}
            </div>

            {/* Scanning Stage */}
            {recognitionStage === 'scanning' && (
              <div style={styles.buttonRow}>
                <button
                  onClick={skipToMenu}
                  style={styles.skipBtn}
                >
                  Skip Recognition (Guest Mode)
                </button>
              </div>
            )}

            {/* Unknown User Stage */}
            {recognitionStage === 'unknown' && (
              <div>
                <div style={styles.inputGroup}>
                  <input
                    type="text"
                    placeholder="Enter your name"
                    value={registrationName}
                    onChange={(e) => setRegistrationName(e.target.value)}
                    onKeyPress={(e) => e.key === 'Enter' && handleTextRegistration()}
                    style={styles.input}
                  />
                  <button
                    onClick={handleTextRegistration}
                    disabled={!registrationName.trim() || isLoading}
                    style={styles.primaryBtn}
                  >
                    Register Face
                  </button>
                </div>
                
                <div style={styles.buttonRow}>
                  <button
                    onClick={retryRecognition}
                    style={styles.secondaryBtn}
                  >
                    Try Again
                  </button>
                  <button
                    onClick={skipToMenu}
                    style={styles.skipBtn}
                  >
                    Continue as Guest
                  </button>
                </div>
              </div>
            )}

            {/* Recognized User Stage */}
            {recognitionStage === 'recognized' && (
              <div>
                <div style={{
                  fontSize: '3rem',
                  marginBottom: '1rem'
                }}>
                  👋
                </div>
                <div style={{
                  fontSize: '1.5rem',
                  fontWeight: '700',
                  color: '#38a169',
                  marginBottom: '2rem'
                }}>
                  Welcome back, {userName}!
                </div>
                <button
                  onClick={proceedToControls}
                  style={styles.successBtn}
                >
                  🚀 Access Robot Controls
                </button>
              </div>
            )}

            {/* Status Information */}
            {status && (
              <div style={styles.statusCard}>
                <h3 style={styles.statusTitle}>🤖 Robot Status</h3>
                <div style={styles.statusGrid}>
                  <div><strong>Camera:</strong> {status.camera_active ? '✅ Active' : '❌ Inactive'}</div>
                  <div><strong>Face Recognition:</strong> {status.face_recognition_available ? '✅ Available' : '❌ Unavailable'}</div>
                  <div><strong>Recognition Attempts:</strong> {status.face_recognition_attempts}/3</div>
                  <div><strong>Current User:</strong> {status.current_user}</div>
                  <div><strong>Awaiting Registration:</strong> {status.awaiting_registration ? 'Yes' : 'No'}</div>
                  <div><strong>Stage:</strong> {recognitionStage}</div>
                </div>
              </div>
            )}
          </>
        )}

        {connectionStatus !== 'connected' && (
          <div>
            <div style={{
              fontSize: '1.1rem',
              color: '#718096',
              marginBottom: '2rem'
            }}>
              {connectionStatus === 'connecting' ? 'Connecting to robot system...' : 'Connection failed'}
            </div>
            
            {connectionStatus === 'disconnected' && (
              <div style={styles.buttonRow}>
                <button
                  onClick={refreshConnection}
                  style={styles.refreshBtn}
                >
                  🔄 Retry Connection
                </button>
                <button
                  onClick={skipToMenu}
                  style={styles.skipBtn}
                >
                  Continue as Guest
                </button>
              </div>
            )}
          </div>
        )}

        <div style={{
          marginTop: '2rem',
          fontSize: '0.9rem',
          color: '#a0aec0',
          borderTop: '1px solid rgba(226, 232, 240, 0.5)',
          paddingTop: '1rem'
        }}>
          🔒 Secure access • Face recognition powered robot control
        </div>
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
    maxWidth: '100%',
    height: 'auto',
    borderRadius: '12px',
    border: '2px solid #e2e8f0'
  },
  message: {
    fontSize: '1.2rem',
    fontWeight: '600',
    color: '#2d3748',
    marginBottom: '2rem',
    minHeight: '2rem'
  },
  inputGroup: {
    display: 'flex',
    gap: '1rem',
    marginBottom: '1rem',
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
    fontWeight: '600',
    transition: 'all 0.3s ease'
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
  successBtn: {
    padding: '1.5rem 3rem',
    borderRadius: '12px',
    border: 'none',
    cursor: 'pointer',
    background: 'linear-gradient(135deg, #48bb78, #38a169)',
    color: 'white',
    fontSize: '1.2rem',
    fontWeight: '600',
    transition: 'all 0.3s ease'
  },
  skipBtn: {
    padding: '0.75rem 1.5rem',
    borderRadius: '8px',
    border: '2px solid #e2e8f0',
    cursor: 'pointer',
    background: 'transparent',
    color: '#718096',
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
    fontWeight: '600',
    transition: 'all 0.3s ease'
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
  statusCard: {
    background: '#f7fafc',
    border: '1px solid #e2e8f0',
    borderRadius: '8px',
    padding: '1rem',
    textAlign: 'left' as const,
    marginTop: '2rem'
  },
  statusTitle: {
    marginTop: 0,
    fontSize: '1.1rem',
    color: '#2d3748'
  },
  statusGrid: {
    display: 'grid',
    gridTemplateColumns: '1fr 1fr',
    gap: '0.5rem',
    fontSize: '0.9rem'
  }
};

export default FaceRecognitionGate;