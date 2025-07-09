import React, { useState, useEffect, useRef } from 'react';

interface FaceRecognitionStatus {
  current_user: string;
  camera_active: boolean;
  face_recognition_available: boolean;
  listening: boolean;
}

type WSMessage =
  | { type: 'status_update'; data: FaceRecognitionStatus }
  | { type: 'user_recognized'; data: { user: string | null; success: boolean } }
  | { type: 'user_registered'; data: { name: string; success: boolean } }
  | { type: 'speech_output'; data: { text: string; success: boolean } }
  | { type: 'error'; data: { message: string } };

const FaceRecognitionGate: React.FC = () => {
  const [ws, setWs] = useState<WebSocket | null>(null);
  const [status, setStatus] = useState<FaceRecognitionStatus | null>(null);
  const [connectionStatus, setConnectionStatus] = useState<'connecting' | 'connected' | 'disconnected'>('connecting');
  const [recognitionStage, setRecognitionStage] = useState<'scanning' | 'unknown' | 'recognized' | 'registering'>('scanning');
  const [userName, setUserName] = useState('');
  const [registrationName, setRegistrationName] = useState('');
  const [message, setMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const cameraRef = useRef<HTMLImageElement>(null);
  const recognitionIntervalRef = useRef<any>(null);
  const scanAttemptsRef = useRef<number>(0); // Use ref instead of state

  // Setup WebSocket connection
  useEffect(() => {
    let reconnectTimer: number;
    
    const connect = () => {
      setConnectionStatus('connecting');
      const websocket = new WebSocket(`ws://${window.location.hostname}:8000/ws`);
      setWs(websocket);

      websocket.onopen = () => {
        console.log('WebSocket connected');
        setConnectionStatus('connected');
        setMessage('Connected to robot - initializing face recognition...');
        startFaceRecognition();
      };

      websocket.onmessage = (event) => {
        try {
          const data: WSMessage = JSON.parse(event.data);
          
          switch (data.type) {
            case 'status_update':
              setStatus(data.data);
              break;
            case 'user_recognized':
              if (data.data.success && data.data.user && data.data.user !== 'Unknown') {
                setUserName(data.data.user);
                setRecognitionStage('recognized');
                setMessage(`Welcome back, ${data.data.user}!`);
                sendWebSocketMessage('speech_command', { text: `Hello ${data.data.user}! Welcome back to the robot control system.` });
                // Clear interval when user is recognized
                if (recognitionIntervalRef.current) {
                  clearInterval(recognitionIntervalRef.current);
                  recognitionIntervalRef.current = null;
                }
                setTimeout(() => {
                  window.location.href = '/menu';
                }, 3000);
              } else {
                // Increment scan attempts and check if we should switch to unknown stage
                scanAttemptsRef.current += 1;
                if (scanAttemptsRef.current >= 5) {
                  setRecognitionStage('unknown');
                  setMessage('I don\'t recognize you. Please tell me your name.');
                  sendWebSocketMessage('speech_command', { text: 'I don\'t recognize you. Please tell me your name.' });
                  // Clear interval when switching to unknown stage
                  if (recognitionIntervalRef.current) {
                    clearInterval(recognitionIntervalRef.current);
                    recognitionIntervalRef.current = null;
                  }
                } else {
                  setMessage(`Scanning for faces... (${scanAttemptsRef.current}/5)`);
                }
              }
              break;
            case 'user_registered':
              if (data.data.success) {
                setUserName(data.data.name);
                setRecognitionStage('recognized');
                setMessage(`Nice to meet you, ${data.data.name}! Registration successful.`);
                sendWebSocketMessage('speech_command', { text: `Nice to meet you ${data.data.name}! You are now registered. Welcome to the robot control system.` });
                setTimeout(() => {
                  window.location.href = '/menu';
                }, 3000);
              } else {
                setMessage('Registration failed. Please try again.');
                setRecognitionStage('unknown');
              }
              setIsLoading(false);
              break;
            case 'error':
              setMessage(`Error: ${data.data.message}`);
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
        setMessage('Connection error - retrying...');
      };

      websocket.onclose = () => {
        console.log('WebSocket closed');
        setConnectionStatus('disconnected');
        setMessage('Disconnected - reconnecting...');
        setWs(null);
        
        if (recognitionIntervalRef.current) {
          clearInterval(recognitionIntervalRef.current);
          recognitionIntervalRef.current = null;
        }
        
        reconnectTimer = window.setTimeout(() => {
          connect();
        }, 3000);
      };
    };

    connect();

    return () => {
      if (reconnectTimer) {
        clearTimeout(reconnectTimer);
      }
      if (recognitionIntervalRef.current) {
        clearInterval(recognitionIntervalRef.current);
      }
      if (ws) {
        ws.close();
      }
    };
  }, []); // Empty dependency array to avoid infinite loops

  // Camera stream setup
  useEffect(() => {
    if (cameraRef.current && connectionStatus === 'connected') {
      const img = cameraRef.current;
      img.src = `http://${window.location.hostname}:8000/api/camera/stream`;
    }
  }, [connectionStatus]);

  const sendWebSocketMessage = (type: string, data: any = {}) => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ type, data }));
    }
  };

  const startFaceRecognition = () => {
    setMessage('Scanning for faces...');
    scanAttemptsRef.current = 0; // Reset scan attempts when starting
    
    // Clear any existing interval
    if (recognitionIntervalRef.current) {
      clearInterval(recognitionIntervalRef.current);
    }
    
    recognitionIntervalRef.current = setInterval(() => {
      sendWebSocketMessage('recognize_user', { mode: 'auto' });
    }, 2000);
  };

  const handleTextRegistration = () => {
    if (registrationName.trim()) {
      setIsLoading(true);
      setMessage('Registering your face...');
      localStorage.setItem('robotUser', registrationName.trim());
      sendWebSocketMessage('register_user', { name: registrationName.trim() });
    }
  };

  const skipToMenu = () => {
    localStorage.setItem('robotUser', 'Guest');
    window.location.href = '/menu';
  };

  const proceedToControls = () => {
    localStorage.setItem('robotUser', userName);
    window.location.href = '/menu';
  };

  const styles = {
    container: {
      minHeight: '100vh',
      width: '100vw',
      background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
      display: 'flex',
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
    connectionStatus: {
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
      marginRight: '10px',
      backgroundColor: connectionStatus === 'connected' ? '#48bb78' : '#e53e3e'
    },
    cameraContainer: {
      marginBottom: '2rem'
    },
    camera: {
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
    button: {
      padding: '1rem 2rem',
      borderRadius: '12px',
      border: 'none',
      cursor: 'pointer',
      fontSize: '1rem',
      fontWeight: '600',
      transition: 'all 0.3s ease'
    },
    primaryBtn: {
      background: 'linear-gradient(135deg, #667eea, #764ba2)',
      color: 'white'
    },
    secondaryBtn: {
      background: 'linear-gradient(135deg, #48bb78, #38a169)',
      color: 'white'
    },
    successBtn: {
      background: 'linear-gradient(135deg, #48bb78, #38a169)',
      color: 'white',
      fontSize: '1.2rem',
      padding: '1.5rem 3rem'
    },
    skipBtn: {
      background: 'transparent',
      color: '#718096',
      border: '2px solid #e2e8f0',
      fontSize: '0.9rem',
      padding: '0.75rem 1.5rem'
    },
    buttonRow: {
      display: 'flex',
      gap: '1rem',
      justifyContent: 'center',
      flexWrap: 'wrap' as const,
      marginBottom: '1rem'
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
        
        <div style={styles.connectionStatus}>
          <div style={styles.statusDot}></div>
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
                style={styles.camera}
              />
            </div>

            <div style={styles.message}>
              {isLoading && <span style={styles.loadingSpinner}></span>}
              {message}
            </div>

            {/* Scanning Stage */}
            {recognitionStage === 'scanning' && (
              <div style={styles.buttonRow}>
                <button
                  onClick={skipToMenu}
                  style={{...styles.button, ...styles.skipBtn}}
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
                    style={{...styles.button, ...styles.primaryBtn}}
                  >
                    Register Face
                  </button>
                </div>
                
                <div style={styles.buttonRow}>
                  <button
                    onClick={skipToMenu}
                    style={{...styles.button, ...styles.skipBtn}}
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
                  style={{...styles.button, ...styles.successBtn}}
                >
                  🚀 Access Robot Controls
                </button>
              </div>
            )}

            {/* Status Information */}
            {status && (
              <div style={{
                marginTop: '2rem',
                padding: '1rem',
                background: 'rgba(240, 244, 248, 0.8)',
                borderRadius: '12px',
                fontSize: '0.9rem',
                color: '#4a5568'
              }}>
                <div><strong>Camera:</strong> {status.camera_active ? '✅ Active' : '❌ Inactive'}</div>
                <div><strong>Face Recognition:</strong> {status.face_recognition_available ? '✅ Available' : '❌ Unavailable'}</div>
                {status.current_user && status.current_user !== 'Unknown' && (
                  <div><strong>Current User:</strong> {status.current_user}</div>
                )}
              </div>
            )}
          </>
        )}

        {connectionStatus !== 'connected' && (
          <div style={{
            fontSize: '1.1rem',
            color: '#718096',
            marginBottom: '2rem'
          }}>
            Connecting to robot system...
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

export default FaceRecognitionGate;