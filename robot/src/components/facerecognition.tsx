// import React, { useState, useEffect, useRef, useCallback } from 'react';
// import { useNavigate } from 'react-router-dom';
// import { setUserName } from './UserHeader';

// interface EnhancedRobotStatus {
//   status: string;
//   current_user: string;
//   camera_active: boolean;
//   face_recognition_available: boolean;
//   face_recognition_attempts: number;
//   awaiting_registration: boolean;
// }

// type WSMessage =
//   | { type: 'status_update'; data: EnhancedRobotStatus }
//   | { type: 'face_recognition_reset'; data: { success: boolean } }
//   | { type: 'error'; data: { message: string } }
//   | { type: 'pong' };

// const FaceRecognitionGate: React.FC = () => {
//   const navigate = useNavigate();
  
//   // Core states
//   const [response, setResponse] = useState<string>('Initializing...');
//   const [status, setStatus] = useState<EnhancedRobotStatus | null>(null);
//   const [connectionStatus, setConnectionStatus] = useState<'connecting' | 'connected' | 'disconnected'>('connecting');
  
//   // Recognition state management - COMPLETELY SEPARATE FROM BACKEND
//   const [recognitionState, setRecognitionState] = useState<'scanning' | 'max_attempts' | 'success'>('scanning');
//   const [localAttempts, setLocalAttempts] = useState(0);
//   const [showUserOptions, setShowUserOptions] = useState(false);

//   // Refs
//   const wsRef = useRef<WebSocket | null>(null);
//   const cameraRef = useRef<HTMLImageElement>(null);
//   const mountedRef = useRef(true);
//   const reconnectTimerRef = useRef<NodeJS.Timeout | null>(null);
//   const reconnectCountRef = useRef(0);
//   const lastBackendAttempts = useRef(0);
//   const wsInitialized = useRef(false); // CRITICAL: Track if WS was initialized

//   // Constants
//   const MAX_RETRIES = 5;
//   const MAX_RECOGNITION_ATTEMPTS = 3;

//   // Cleanup on unmount
//   useEffect(() => {
//     mountedRef.current = true;
//     return () => {
//       mountedRef.current = false;
//       cleanup();
//     };
//   }, []);

//   const cleanup = useCallback(() => {
//     if (reconnectTimerRef.current) {
//       clearTimeout(reconnectTimerRef.current);
//       reconnectTimerRef.current = null;
//     }
//     if (wsRef.current) {
//       try {
//         wsRef.current.close();
//       } catch (e) {
//         console.warn('Error closing WebSocket:', e);
//       }
//       wsRef.current = null;
//     }
//     wsInitialized.current = false;
//   }, []);

//   // STABLE WebSocket message handler
//   const handleWebSocketMessage = useCallback((event: MessageEvent) => {
//     if (!mountedRef.current) return;
    
//     try {
//       const data: WSMessage = JSON.parse(event.data);
//       console.log('📥 Face Recognition received:', data.type, data);
      
//       switch (data.type) {
//         case 'status_update':
//           setStatus(data.data);
          
//           // Track backend attempts to detect changes
//           const backendAttempts = data.data.face_recognition_attempts || 0;
          
//           // SUCCESS CASE: User recognized
//           if (data.data.current_user && 
//               data.data.current_user !== 'Unknown' && 
//               data.data.current_user !== '') {
            
//             console.log('✅ USER RECOGNIZED:', data.data.current_user);
//             setUserName(data.data.current_user);
//             setRecognitionState('success');
//             setResponse(`Welcome back, ${data.data.current_user}! Redirecting...`);
            
//             // Navigate after delay
//             setTimeout(() => {
//               if (mountedRef.current) {
//                 navigate('/menu');
//               }
//             }, 2000);
//             return;
//           }
          
//           // ATTEMPT TRACKING: Only update if backend attempts increased
//           if (backendAttempts > lastBackendAttempts.current) {
//             setLocalAttempts(backendAttempts);
//             lastBackendAttempts.current = backendAttempts;
            
//             if (backendAttempts >= MAX_RECOGNITION_ATTEMPTS) {
//               // MAX ATTEMPTS REACHED - Show options, DON'T auto-navigate
//               console.log('❌ MAX ATTEMPTS REACHED - showing user options');
//               setRecognitionState('max_attempts');
//               setShowUserOptions(true);
//               setResponse('Face recognition failed after 3 attempts. What would you like to do?');
//             } else {
//               // Still attempting
//               setRecognitionState('scanning');
//               setResponse(`Scanning for faces... (${backendAttempts}/${MAX_RECOGNITION_ATTEMPTS})`);
//             }
//           } else if (backendAttempts === 0 && localAttempts > 0) {
//             // Reset detected
//             console.log('🔄 Recognition reset detected');
//             setLocalAttempts(0);
//             setRecognitionState('scanning');
//             setShowUserOptions(false);
//             setResponse('Face recognition reset. Scanning...');
//           }
//           break;
          
//         case 'face_recognition_reset':
//           if (data.data.success) {
//             console.log('✅ Face recognition reset confirmed');
//             setLocalAttempts(0);
//             lastBackendAttempts.current = 0;
//             setRecognitionState('scanning');
//             setShowUserOptions(false);
//             setResponse('Face recognition reset. Starting new scan...');
//           }
//           break;
          
//         case 'error':
//           setResponse(`Error: ${data.data.message}`);
//           break;
//       }
//     } catch (error) {
//       console.error('Message parse error:', error);
//     }
//   }, [navigate, localAttempts]);

//   // STABLE WebSocket connection handlers
//   const handleWebSocketOpen = useCallback(() => {
//     if (!mountedRef.current) return;
//     console.log('✅ Face Recognition WebSocket connected');
//     setConnectionStatus('connected');
//     setResponse('Connected. Scanning for faces...');
//     reconnectCountRef.current = 0;
//   }, []);

//   const handleWebSocketError = useCallback(() => {
//     if (!mountedRef.current) return;
//     console.error('Face Recognition WebSocket error');
//     setConnectionStatus('disconnected');
//     setResponse('Connection error. Retrying...');
//   }, []);

//   const handleWebSocketClose = useCallback(() => {
//     if (!mountedRef.current) return;
    
//     console.log('Face Recognition WebSocket closed');
//     setConnectionStatus('disconnected');
//     setResponse('Connection lost. Reconnecting...');
//     wsRef.current = null;
    
//     // Exponential backoff reconnection
//     if (reconnectCountRef.current < MAX_RETRIES) {
//       reconnectCountRef.current++;
//       const delay = Math.min(3000 * Math.pow(1.5, reconnectCountRef.current - 1), 15000);
      
//       reconnectTimerRef.current = setTimeout(() => {
//         if (mountedRef.current && !wsInitialized.current) {
//           connectWebSocket();
//         }
//       }, delay);
//     } else {
//       setResponse('Connection failed after maximum retries. Please refresh the page.');
//     }
//   }, []);

//   // STABLE WebSocket connection function
//   const connectWebSocket = useCallback(() => {
//     if (!mountedRef.current || wsInitialized.current) return;
    
//     cleanup();
//     setConnectionStatus('connecting');
//     setResponse('Connecting to robot...');
    
//     try {
//       const ws = new WebSocket(`ws://${window.location.hostname}:8000/ws`);
//       wsRef.current = ws;
//       wsInitialized.current = true; // Mark as initialized

//       ws.addEventListener('open', handleWebSocketOpen);
//       ws.addEventListener('message', handleWebSocketMessage);
//       ws.addEventListener('error', handleWebSocketError);
//       ws.addEventListener('close', handleWebSocketClose);
      
//     } catch (error) {
//       console.error('Failed to create WebSocket:', error);
//       setConnectionStatus('disconnected');
//       setResponse('Failed to connect to robot');
//       wsInitialized.current = false;
//     }
//   }, [handleWebSocketOpen, handleWebSocketMessage, handleWebSocketError, handleWebSocketClose, cleanup]);

//   // Initialize WebSocket connection ONCE - CRITICAL FIX
//   useEffect(() => {
//     if (!wsInitialized.current) {
//       connectWebSocket();
//     }
//   }, []); // EMPTY dependency array - only run once

//   // Camera setup - SEPARATE EFFECT with minimal dependencies
//   useEffect(() => {
//     if (cameraRef.current && connectionStatus === 'connected') {
//       const img = cameraRef.current;
//       const timestamp = Date.now();
//       img.src = `http://${window.location.hostname}:8000/api/camera/stream?t=${timestamp}`;
      
//       const handleError = () => {
//         console.warn('Camera stream not available');
//       };
      
//       const handleLoad = () => {
//         console.log('✅ Camera stream loaded');
//       };
      
//       img.addEventListener('error', handleError);
//       img.addEventListener('load', handleLoad);
      
//       return () => {
//         img.removeEventListener('error', handleError);
//         img.removeEventListener('load', handleLoad);
//       };
//     }
//   }, [connectionStatus]);

//   // Action functions - STABLE
//   const sendWebSocketMessage = useCallback((type: string, data: any = {}) => {
//     if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
//       console.log('📤 Sending:', type, data);
//       wsRef.current.send(JSON.stringify({ type, data }));
//       return true;
//     } else {
//       console.warn('WebSocket not connected, cannot send message');
//       setResponse('Not connected to robot');
//       return false;
//     }
//   }, []);

//   const resetRecognition = useCallback(() => {
//     console.log('🔄 RESETTING FACE RECOGNITION');
//     setResponse('Resetting face recognition...');
    
//     // Reset local state immediately
//     setLocalAttempts(0);
//     lastBackendAttempts.current = 0;
//     setRecognitionState('scanning');
//     setShowUserOptions(false);
    
//     // Send reset message to backend
//     sendWebSocketMessage('reset_face_recognition');
//   }, [sendWebSocketMessage]);

//   const continueAsGuest = useCallback(() => {
//     console.log('👤 CONTINUING AS GUEST');
//     setUserName('Guest');
//     navigate('/menu');
//   }, [navigate]);

//   // FIXED: Go to registration without WebSocket dependency issues
//   const goToRegistration = useCallback(() => {
//     console.log('📝 GOING TO REGISTRATION');
//     // Don't close WebSocket, just navigate
//     navigate('/register');
//   }, [navigate]);

//   // Render helper functions
//   const renderConnectionStatus = () => (
//     <div style={styles.connectionIndicator}>
//       <div style={{
//         ...styles.statusDot,
//         backgroundColor: connectionStatus === 'connected' ? '#48bb78' : 
//                         connectionStatus === 'connecting' ? '#ed8936' : '#e53e3e'
//       }}></div>
//       <span>
//         {connectionStatus === 'connected' ? 'Connected' : 
//          connectionStatus === 'connecting' ? 'Connecting...' : 'Disconnected'}
//       </span>
//     </div>
//   );

//   const renderCamera = () => (
//     <div style={styles.cameraContainer}>
//       <img
//         ref={cameraRef}
//         alt="Robot Camera Feed"
//         style={styles.cameraFeed}
//       />
//     </div>
//   );

//   const renderScanningState = () => (
//     <>
//       <div style={styles.infoBox}>
//         👁️ <strong>Face Recognition Active</strong><br />
//         Please look at the camera for identification
//         {localAttempts > 0 && (
//           <><br />Attempts: {localAttempts}/{MAX_RECOGNITION_ATTEMPTS}</>
//         )}
//       </div>
      
//       <div style={styles.buttonRow}>
//         <button 
//           onClick={resetRecognition} 
//           style={styles.secondaryBtn}
//           disabled={connectionStatus !== 'connected'}
//         >
//           🔄 Reset & Try Again
//         </button>
//         <button 
//           onClick={continueAsGuest} 
//           style={styles.guestBtn}
//         >
//           👤 Continue as Guest
//         </button>
//       </div>
//     </>
//   );

//   const renderMaxAttemptsState = () => (
//     <div style={styles.optionsContainer}>
//       <div style={styles.warningBox}>
//         ❌ <strong>Recognition Failed</strong><br />
//         Face recognition failed after {MAX_RECOGNITION_ATTEMPTS} attempts. Please choose an option:
//       </div>
      
//       <div style={styles.buttonGrid}>
//         <button 
//           onClick={resetRecognition} 
//           style={styles.primaryBtn}
//           disabled={connectionStatus !== 'connected'}
//         >
//           🔄 Try Again ({MAX_RECOGNITION_ATTEMPTS} more attempts)
//         </button>
//         <button 
//           onClick={goToRegistration} 
//           style={styles.registerBtn}
//         >
//           📝 Register as New User
//         </button>
//         <button 
//           onClick={continueAsGuest} 
//           style={styles.guestBtn}
//         >
//           👤 Continue as Guest
//         </button>
//       </div>
//     </div>
//   );

//   const renderSuccessState = () => (
//     <div style={styles.successBox}>
//       ✅ <strong>Recognition Successful!</strong><br />
//       Redirecting to main menu...
//     </div>
//   );

//   const renderMainContent = () => {
//     if (connectionStatus === 'disconnected') {
//       return (
//         <div style={styles.disconnectedContainer}>
//           <div style={styles.warningBox}>
//             ⚠️ <strong>Connection Lost</strong><br />
//             Unable to connect to robot. Please check your connection.
//           </div>
//           <div style={styles.buttonRow}>
//             <button 
//               onClick={() => {
//                 wsInitialized.current = false;
//                 connectWebSocket();
//               }} 
//               style={styles.primaryBtn}
//             >
//               🔄 Retry Connection
//             </button>
//             <button 
//               onClick={continueAsGuest} 
//               style={styles.guestBtn}
//             >
//               👤 Continue as Guest
//             </button>
//           </div>
//         </div>
//       );
//     }

//     switch (recognitionState) {
//       case 'scanning':
//         return renderScanningState();
//       case 'max_attempts':
//         return renderMaxAttemptsState();
//       case 'success':
//         return renderSuccessState();
//       default:
//         return renderScanningState();
//     }
//   };

//   return (
//     <div style={styles.container}>
//       <div style={styles.card}>
//         <h1 style={styles.title}>🤖 Robot Access Control</h1>
//         <p style={styles.subtitle}>Face recognition security system</p>
        
//         {renderConnectionStatus()}
//         {renderCamera()}

//         <div style={styles.message}>
//           {response}
//         </div>

//         {renderMainContent()}

//         {/* Debug Info - Remove in production */}
//         {process.env.NODE_ENV === 'development' && status && (
//           <div style={styles.debugBox}>
//             <strong>Debug Info:</strong><br />
//             Frontend State: {recognitionState}<br />
//             Local Attempts: {localAttempts}/{MAX_RECOGNITION_ATTEMPTS}<br />
//             Show Options: {showUserOptions ? 'Yes' : 'No'}<br />
//             WS Status: {connectionStatus}<br />
//             WS Initialized: {wsInitialized.current ? 'Yes' : 'No'}<br />
//             Backend User: {status.current_user}<br />
//             Backend Attempts: {status.face_recognition_attempts}<br />
//             Awaiting Registration: {status.awaiting_registration ? 'Yes' : 'No'}<br />
//             Camera: {status.camera_active ? 'Active' : 'Inactive'}
//           </div>
//         )}
//       </div>
//     </div>
//   );
// };

// const styles = {
//   container: {
//     minHeight: '100vh',
//     width: '100vw',
//     background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
//     display: 'flex' as const,
//     justifyContent: 'center',
//     alignItems: 'center',
//     fontFamily: '"Inter", sans-serif',
//     padding: '2rem'
//   },
//   card: {
//     background: 'rgba(255, 255, 255, 0.95)',
//     backdropFilter: 'blur(20px)',
//     borderRadius: '24px',
//     padding: '3rem',
//     boxShadow: '0 20px 40px rgba(0, 0, 0, 0.1)',
//     border: '1px solid rgba(255, 255, 255, 0.2)',
//     textAlign: 'center' as const,
//     width: '100%',
//     maxWidth: '600px'
//   },
//   title: {
//     fontSize: '2.5rem',
//     fontWeight: '700',
//     color: '#2d3748',
//     marginBottom: '1rem',
//     background: 'linear-gradient(135deg, #667eea, #764ba2)',
//     WebkitBackgroundClip: 'text',
//     WebkitTextFillColor: 'transparent',
//     backgroundClip: 'text'
//   },
//   subtitle: {
//     color: '#718096',
//     marginBottom: '2rem',
//     fontSize: '1.2rem'
//   },
//   connectionIndicator: {
//     display: 'flex',
//     alignItems: 'center',
//     justifyContent: 'center',
//     marginBottom: '1rem',
//     fontSize: '0.9rem'
//   },
//   statusDot: {
//     width: '8px',
//     height: '8px',
//     borderRadius: '50%',
//     marginRight: '8px'
//   },
//   cameraContainer: {
//     textAlign: 'center' as const,
//     marginBottom: '2rem'
//   },
//   cameraFeed: {
//     width: '100%',
//     maxWidth: '400px',
//     height: 'auto',
//     borderRadius: '16px',
//     border: '3px solid #e2e8f0',
//     boxShadow: '0 8px 16px rgba(0, 0, 0, 0.1)',
//     minHeight: '300px',
//     backgroundColor: '#f7fafc'
//   },
//   message: {
//     fontSize: '1.2rem',
//     fontWeight: '600',
//     color: '#2d3748',
//     marginBottom: '2rem',
//     minHeight: '2rem',
//     padding: '1rem',
//     background: 'rgba(247, 250, 252, 0.8)',
//     borderRadius: '12px'
//   },
//   infoBox: {
//     fontSize: '1.1rem',
//     color: '#4a5568',
//     marginBottom: '2rem',
//     padding: '1rem',
//     background: 'rgba(219, 234, 254, 0.8)',
//     borderRadius: '12px',
//     border: '1px solid rgba(59, 130, 246, 0.3)'
//   },
//   warningBox: {
//     fontSize: '1.1rem',
//     color: '#4a5568',
//     marginBottom: '2rem',
//     padding: '1rem',
//     background: 'rgba(254, 215, 170, 0.8)',
//     borderRadius: '12px',
//     border: '1px solid rgba(251, 146, 60, 0.4)'
//   },
//   successBox: {
//     fontSize: '1.1rem',
//     color: '#4a5568',
//     marginBottom: '2rem',
//     padding: '1rem',
//     background: 'rgba(209, 250, 229, 0.8)',
//     borderRadius: '12px',
//     border: '1px solid rgba(52, 211, 153, 0.4)'
//   },
//   optionsContainer: {
//     marginBottom: '2rem'
//   },
//   disconnectedContainer: {
//     marginBottom: '2rem'
//   },
//   buttonGrid: {
//     display: 'grid',
//     gridTemplateColumns: '1fr',
//     gap: '1rem',
//     marginBottom: '1rem'
//   },
//   buttonRow: {
//     display: 'flex',
//     gap: '0.5rem',
//     justifyContent: 'center',
//     flexWrap: 'wrap' as const,
//     marginBottom: '1rem'
//   },
//   primaryBtn: {
//     padding: '1rem 2rem',
//     borderRadius: '12px',
//     border: 'none',
//     cursor: 'pointer',
//     background: 'linear-gradient(135deg, #48bb78, #38a169)',
//     color: 'white',
//     fontSize: '1rem',
//     fontWeight: '600',
//     transition: 'all 0.3s ease',
//     disabled: {
//       opacity: 0.6,
//       cursor: 'not-allowed'
//     }
//   },
//   secondaryBtn: {
//     padding: '0.75rem 1.5rem',
//     borderRadius: '8px',
//     border: 'none',
//     cursor: 'pointer',
//     background: 'linear-gradient(135deg, #48bb78, #38a169)',
//     color: 'white',
//     fontSize: '0.9rem',
//     fontWeight: '600',
//     transition: 'all 0.3s ease'
//   },
//   guestBtn: {
//     padding: '0.75rem 1.5rem',
//     borderRadius: '8px',
//     border: '2px solid #667eea',
//     cursor: 'pointer',
//     background: 'transparent',
//     color: '#667eea',
//     fontSize: '0.9rem',
//     fontWeight: '600',
//     transition: 'all 0.3s ease'
//   },
//   registerBtn: {
//     padding: '1rem 2rem',
//     borderRadius: '12px',
//     border: 'none',
//     cursor: 'pointer',
//     background: 'linear-gradient(135deg, #667eea, #764ba2)',
//     color: 'white',
//     fontSize: '1rem',
//     fontWeight: '600',
//     transition: 'all 0.3s ease'
//   },
//   debugBox: {
//     marginTop: '2rem',
//     padding: '1rem',
//     background: 'rgba(240, 244, 248, 0.8)',
//     borderRadius: '8px',
//     fontSize: '0.8rem',
//     color: '#4a5568',
//     textAlign: 'left' as const
//   }
// };

// export default FaceRecognitionGate;
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
  const [response, setResponse] = useState<string>('Initializing...');
  const [status, setStatus] = useState<EnhancedRobotStatus | null>(null);
  const [connectionStatus, setConnectionStatus] = useState<'connecting' | 'connected' | 'disconnected'>('connecting');

  // Registration states
  const [registrationName, setRegistrationName] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  // Refs
  const wsRef = useRef<WebSocket | null>(null);
  const cameraRef = useRef<HTMLImageElement>(null);
  const mountedRef = useRef(true);
  const reconnectTimerRef = useRef<NodeJS.Timeout | null>(null);
  const connectionAttempts = useRef(0);

  // Constants
  const MAX_CONNECTION_ATTEMPTS = 3;

  // Cleanup on unmount
  useEffect(() => {
    mountedRef.current = true;
    return () => {
      mountedRef.current = false;
      if (reconnectTimerRef.current) {
        clearTimeout(reconnectTimerRef.current);
      }
      if (wsRef.current) {
        wsRef.current.close();
        wsRef.current = null;
      }
    };
  }, []);

  // SIMPLIFIED message handler
  const handleMessage = useCallback((event: MessageEvent) => {
    if (!mountedRef.current) return;
    
    try {
      const data: WSMessage = JSON.parse(event.data);
      
      switch (data.type) {
        case 'status_update':
          setStatus(data.data);
          
          // If user was recognized during registration, go to menu
          if (data.data.current_user && 
              data.data.current_user !== 'Unknown' && 
              data.data.current_user !== '') {
            console.log('✅ USER RECOGNIZED DURING REGISTRATION:', data.data.current_user);
            setUserName(data.data.current_user);
            setResponse(`Welcome back, ${data.data.current_user}!`);
            
            setTimeout(() => {
              if (mountedRef.current) {
                navigate('/menu');
              }
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
              if (mountedRef.current) {
                navigate('/menu');
              }
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

  // SIMPLIFIED WebSocket connection - NO COMPLEX RECONNECTION
  const connectWebSocket = useCallback(() => {
    if (!mountedRef.current) return;
    
    // Close existing connection
    if (wsRef.current) {
      wsRef.current.close();
      wsRef.current = null;
    }
    
    if (connectionAttempts.current >= MAX_CONNECTION_ATTEMPTS) {
      console.log('❌ Max connection attempts reached');
      setConnectionStatus('disconnected');
      setResponse('Connection failed. You can still register without live camera feed.');
      return;
    }
    
    connectionAttempts.current++;
    setConnectionStatus('connecting');
    setResponse(`Connecting to robot... (attempt ${connectionAttempts.current}/${MAX_CONNECTION_ATTEMPTS})`);
    
    try {
      const ws = new WebSocket(`ws://${window.location.hostname}:8000/ws`);
      wsRef.current = ws;

      // Connection opened
      ws.onopen = () => {
        if (!mountedRef.current) return;
        console.log('✅ Registration WebSocket connected');
        setConnectionStatus('connected');
        setResponse('Connected - ready for registration');
        connectionAttempts.current = 0; // Reset on success
      };

      // Message received
      ws.onmessage = handleMessage;

      // Connection error
      ws.onerror = () => {
        if (!mountedRef.current) return;
        console.error('❌ Registration WebSocket error');
      };

      // Connection closed
      ws.onclose = () => {
        if (!mountedRef.current) return;
        console.log('🔌 Registration WebSocket closed');
        wsRef.current = null;
        
        // Only auto-reconnect if we haven't reached max attempts
        if (connectionAttempts.current < MAX_CONNECTION_ATTEMPTS) {
          setConnectionStatus('connecting');
          setResponse('Connection lost. Retrying...');
          
          reconnectTimerRef.current = setTimeout(() => {
            if (mountedRef.current) {
              connectWebSocket();
            }
          }, 2000);
        } else {
          setConnectionStatus('disconnected');
          setResponse('Connection failed. You can still register without live features.');
        }
      };
      
      // Connection timeout
      setTimeout(() => {
        if (ws.readyState === WebSocket.CONNECTING) {
          console.log('⏰ Registration WebSocket connection timeout');
          ws.close();
        }
      }, 10000); // 10 second timeout
      
    } catch (error) {
      console.error('Failed to create WebSocket:', error);
      setConnectionStatus('disconnected');
      setResponse('Failed to connect to robot');
    }
  }, [handleMessage]);

  // Initialize WebSocket ONCE
  useEffect(() => {
    connectWebSocket();
  }, []); // Empty dependency array - only run once

  // Camera setup
  useEffect(() => {
    if (cameraRef.current && connectionStatus === 'connected') {
      const img = cameraRef.current;
      const timestamp = Date.now();
      img.src = `http://${window.location.hostname}:8000/api/camera/stream?t=${timestamp}`;
      
      img.onerror = () => {
        console.warn('Camera stream not available');
      };
      
      img.onload = () => {
        console.log('✅ Camera stream loaded successfully');
      };
    }
  }, [connectionStatus]);

  // Send WebSocket message
  const sendMessage = useCallback((type: string, data: any = {}) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      console.log('📤 Registration sending:', type, data);
      wsRef.current.send(JSON.stringify({ type, data }));
    } else {
      setResponse('Not connected - registration may not work optimally');
    }
  }, []);

  // Action functions
  const handleRegistration = useCallback(() => {
    if (registrationName.trim()) {
      setIsLoading(true);
      setResponse('📝 Registering your face...');
      
      if (connectionStatus === 'connected') {
        sendMessage('register_user', { name: registrationName.trim() });
      } else {
        // Fallback: try HTTP registration if WebSocket is not connected
        setResponse('⚠️ WebSocket not connected. Trying HTTP registration...');
        
        fetch(`http://${window.location.hostname}:8000/api/register-user`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({ name: registrationName.trim() }),
        })
        .then(response => response.json())
        .then(data => {
          setIsLoading(false);
          if (data.success) {
            setUserName(registrationName.trim());
            setResponse(`Welcome, ${registrationName.trim()}! Registration successful.`);
            setTimeout(() => {
              if (mountedRef.current) {
                navigate('/menu');
              }
            }, 2000);
          } else {
            setResponse('❌ Registration failed. Please try again.');
          }
        })
        .catch(error => {
          setIsLoading(false);
          setResponse('❌ Registration failed. Network error.');
          console.error('Registration error:', error);
        });
      }
    } else {
      setResponse('❌ Please enter your name');
    }
  }, [registrationName, connectionStatus, sendMessage, navigate]);

  const goBackToRecognition = useCallback(() => {
    console.log('🔄 GOING BACK TO FACE RECOGNITION');
    if (connectionStatus === 'connected') {
      sendMessage('reset_face_recognition');
    }
    setTimeout(() => {
      navigate('/');
    }, 1000);
  }, [navigate, connectionStatus, sendMessage]);

  const skipToMenu = useCallback(() => {
    setUserName('Guest');
    navigate('/menu');
  }, [navigate]);

  const retryConnection = useCallback(() => {
    connectionAttempts.current = 0; // Reset attempts
    connectWebSocket();
  }, [connectWebSocket]);

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
             connectionStatus === 'connecting' ? 'Connecting...' : 'Offline Mode'}
          </span>
        </div>

        {/* Camera - show regardless of connection status */}
        <div style={styles.cameraContainer}>
          <img
            ref={cameraRef}
            alt="Robot Camera Feed"
            style={styles.cameraFeed}
            onError={() => console.warn('Camera image failed to load')}
            onLoad={() => console.log('Camera image loaded')}
          />
          {connectionStatus !== 'connected' && (
            <div style={styles.cameraOverlay}>
              📷 Camera feed unavailable<br />
              <small>Registration will work without live preview</small>
            </div>
          )}
        </div>

        <div style={styles.message}>
          {isLoading && <span style={styles.loadingSpinner}></span>}
          {response}
        </div>

        <div style={styles.warningBox}>
          🆔 <strong>New User Registration</strong><br />
          Please enter your name and register your face for future access.
        </div>

        {/* Registration form - always available */}
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
          • Look directly at the camera (if available)<br />
          • Ensure good lighting<br />
          • Keep your face visible and centered<br />
          • Registration works even without live camera feed
        </div>
        
        <div style={styles.buttonRow}>
          <button 
            onClick={goBackToRecognition} 
            style={styles.secondaryBtn}
            disabled={isLoading}
          >
            🔄 Try Face Recognition Again
          </button>
          
          {connectionStatus === 'disconnected' && (
            <button 
              onClick={retryConnection} 
              style={styles.refreshBtn}
              disabled={isLoading}
            >
              🔌 Retry Connection
            </button>
          )}
          
          <button 
            onClick={skipToMenu} 
            style={styles.skipBtn}
            disabled={isLoading}
          >
            👤 Skip & Continue as Guest
          </button>
        </div>

        {/* Debug Info */}
        {process.env.NODE_ENV === 'development' && (
          <div style={styles.debugBox}>
            <strong>Registration Debug:</strong><br />
            WS Status: {connectionStatus}<br />
            Connection Attempts: {connectionAttempts.current}/{MAX_CONNECTION_ATTEMPTS}<br />
            {status && (
              <>
                Backend User: {status.current_user}<br />
                Backend Attempts: {status.face_recognition_attempts}<br />
                Awaiting Registration: {status.awaiting_registration ? 'Yes' : 'No'}<br />
                Camera: {status.camera_active ? 'Active' : 'Inactive'}<br />
              </>
            )}
            Response: {response}
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
    fontSize: '1rem',
    fontWeight: '600'
  },
  statusDot: {
    width: '12px',
    height: '12px',
    borderRadius: '50%',
    marginRight: '10px'
  },
  cameraContainer: {
    position: 'relative' as const,
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
  cameraOverlay: {
    position: 'absolute' as const,
    top: '50%',
    left: '50%',
    transform: 'translate(-50%, -50%)',
    background: 'rgba(0, 0, 0, 0.7)',
    color: 'white',
    padding: '1rem',
    borderRadius: '8px',
    fontSize: '1rem',
    textAlign: 'center' as const
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