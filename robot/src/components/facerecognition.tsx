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
  | { type: 'face_recognition_reset'; data: { success: boolean } }
  | { type: 'error'; data: { message: string } }
  | { type: 'pong' };

const FaceRecognitionGate: React.FC = () => {
  const navigate = useNavigate();
  
  // Core states
  const [response, setResponse] = useState<string>('Initializing...');
  const [status, setStatus] = useState<EnhancedRobotStatus | null>(null);
  const [connectionStatus, setConnectionStatus] = useState<'connecting' | 'connected' | 'disconnected'>('connecting');
  
  // Recognition state management
  const [recognitionState, setRecognitionState] = useState<'scanning' | 'max_attempts' | 'success'>('scanning');
  const [localAttempts, setLocalAttempts] = useState(0);

  // Refs
  const wsRef = useRef<WebSocket | null>(null);
  const cameraRef = useRef<HTMLImageElement>(null);
  const mountedRef = useRef(true);
  const reconnectTimerRef = useRef<NodeJS.Timeout | null>(null);
  const lastBackendAttempts = useRef(0);
  const connectionAttempts = useRef(0);

  // Constants
  const MAX_RECOGNITION_ATTEMPTS = 3;
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

  // SIMPLIFIED WebSocket message handler
  const handleMessage = useCallback((event: MessageEvent) => {
    if (!mountedRef.current) return;
    
    try {
      const data: WSMessage = JSON.parse(event.data);
      
      switch (data.type) {
        case 'status_update':
          setStatus(data.data);
          
          const backendAttempts = data.data.face_recognition_attempts || 0;
          
          // SUCCESS: User recognized
          if (data.data.current_user && data.data.current_user !== 'Unknown' && data.data.current_user !== '') {
            console.log('✅ USER RECOGNIZED:', data.data.current_user);
            setUserName(data.data.current_user);
            setRecognitionState('success');
            setResponse(`Welcome back, ${data.data.current_user}! Redirecting...`);
            
            setTimeout(() => {
              if (mountedRef.current) {
                navigate('/menu');
              }
            }, 2000);
            return;
          }
          
          // Track attempts
          if (backendAttempts > lastBackendAttempts.current) {
            setLocalAttempts(backendAttempts);
            lastBackendAttempts.current = backendAttempts;
            
            if (backendAttempts >= MAX_RECOGNITION_ATTEMPTS) {
              setRecognitionState('max_attempts');
              setResponse('Face recognition failed after 3 attempts. Choose an option below:');
            } else {
              setRecognitionState('scanning');
              setResponse(`Scanning for faces... (${backendAttempts}/${MAX_RECOGNITION_ATTEMPTS})`);
            }
          } else if (backendAttempts === 0 && localAttempts > 0) {
            // Reset detected
            setLocalAttempts(0);
            setRecognitionState('scanning');
            setResponse('Face recognition reset. Scanning...');
          }
          break;
          
        case 'face_recognition_reset':
          if (data.data.success) {
            setLocalAttempts(0);
            lastBackendAttempts.current = 0;
            setRecognitionState('scanning');
            setResponse('Face recognition reset. Starting new scan...');
          }
          break;
          
        case 'error':
          setResponse(`Error: ${data.data.message}`);
          break;
      }
    } catch (error) {
      console.error('Message parse error:', error);
    }
  }, [navigate, localAttempts]);

  // SIMPLIFIED WebSocket connection - NO AUTOMATIC RECONNECTION
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
      setResponse('Connection failed after 3 attempts. You can continue as guest or try again.');
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
        console.log('✅ WebSocket connected');
        setConnectionStatus('connected');
        setResponse('Connected. Scanning for faces...');
        connectionAttempts.current = 0; // Reset on success
      };

      // Message received
      ws.onmessage = handleMessage;

      // Connection error
      ws.onerror = () => {
        if (!mountedRef.current) return;
        console.error('❌ WebSocket error');
        setConnectionStatus('disconnected');
        setResponse('Connection error occurred');
      };

      // Connection closed
      ws.onclose = () => {
        if (!mountedRef.current) return;
        console.log('🔌 WebSocket closed');
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
          setResponse('Connection lost. Please try again or continue as guest.');
        }
      };
      
      // Connection timeout
      setTimeout(() => {
        if (ws.readyState === WebSocket.CONNECTING) {
          console.log('⏰ WebSocket connection timeout');
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
    }
  }, [connectionStatus]);

  // Send WebSocket message
  const sendMessage = useCallback((type: string, data: any = {}) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ type, data }));
      return true;
    } else {
      setResponse('Not connected to robot');
      return false;
    }
  }, []);

  // Action functions
  const resetRecognition = useCallback(() => {
    setResponse('Resetting face recognition...');
    setLocalAttempts(0);
    lastBackendAttempts.current = 0;
    setRecognitionState('scanning');
    sendMessage('reset_face_recognition');
  }, [sendMessage]);

  const continueAsGuest = useCallback(() => {
    setUserName('Guest');
    navigate('/menu');
  }, [navigate]);

  const goToRegistration = useCallback(() => {
    // SIMPLE NAVIGATION - No WebSocket complexity
    console.log('📝 Navigating to registration');
    navigate('/register');
  }, [navigate]);

  const retryConnection = useCallback(() => {
    connectionAttempts.current = 0; // Reset attempts
    connectWebSocket();
  }, [connectWebSocket]);

  // Render functions
  const renderConnectionStatus = () => (
    <div style={styles.connectionIndicator}>
      <div style={{
        ...styles.statusDot,
        backgroundColor: connectionStatus === 'connected' ? '#48bb78' : 
                        connectionStatus === 'connecting' ? '#ed8936' : '#e53e3e'
      }}></div>
      <span>
        {connectionStatus === 'connected' ? 'Connected & Ready' : 
         connectionStatus === 'connecting' ? 'Connecting...' : 'Connection Failed'}
      </span>
    </div>
  );

  const renderCamera = () => (
    <div style={styles.cameraContainer}>
      <img
        ref={cameraRef}
        alt="Robot Camera Feed"
        style={styles.cameraFeed}
      />
    </div>
  );

  const renderContent = () => {
    // Always show options if connection failed OR max attempts reached
    if (connectionStatus === 'disconnected' || recognitionState === 'max_attempts') {
      return (
        <div style={styles.optionsContainer}>
          {connectionStatus === 'disconnected' ? (
            <div style={styles.warningBox}>
              ⚠️ <strong>Connection Failed</strong><br />
              Unable to connect to robot. Choose an option below:
            </div>
          ) : (
            <div style={styles.warningBox}>
              ❌ <strong>Recognition Failed</strong><br />
              Face recognition failed after {MAX_RECOGNITION_ATTEMPTS} attempts. Choose an option below:
            </div>
          )}
          
          <div style={styles.buttonGrid}>
            {connectionStatus === 'disconnected' ? (
              <button onClick={retryConnection} style={styles.primaryBtn}>
                🔄 Retry Connection
              </button>
            ) : (
              <button onClick={resetRecognition} style={styles.primaryBtn}>
                🔄 Try Face Recognition Again
              </button>
            )}
            
            <button onClick={goToRegistration} style={styles.registerBtn}>
              📝 Register as New User
            </button>
            
            <button onClick={continueAsGuest} style={styles.guestBtn}>
              👤 Continue as Guest
            </button>
          </div>
        </div>
      );
    }

    if (recognitionState === 'success') {
      return (
        <div style={styles.successBox}>
          ✅ <strong>Recognition Successful!</strong><br />
          Redirecting to main menu...
        </div>
      );
    }

    // Default scanning state
    return (
      <div style={styles.infoBox}>
        👁️ <strong>Face Recognition Active</strong><br />
        Please look at the camera for identification
        {localAttempts > 0 && (
          <><br />Attempts: {localAttempts}/{MAX_RECOGNITION_ATTEMPTS}</>
        )}
        <div style={styles.buttonRow}>
          <button onClick={resetRecognition} style={styles.secondaryBtn}>
            🔄 Reset & Try Again
          </button>
          <button onClick={continueAsGuest} style={styles.guestBtn}>
            👤 Continue as Guest
          </button>
        </div>
      </div>
    );
  };

  return (
    <div style={styles.container}>
      <div style={styles.card}>
        <h1 style={styles.title}>🤖 Robot Access Control</h1>
        <p style={styles.subtitle}>Face recognition security system</p>
        
        {renderConnectionStatus()}
        {renderCamera()}

        <div style={styles.message}>
          {response}
        </div>

        {renderContent()}

        {/* Debug Info */}
        {process.env.NODE_ENV === 'development' && (
          <div style={styles.debugBox}>
            <strong>Debug Info:</strong><br />
            Frontend State: {recognitionState}<br />
            Local Attempts: {localAttempts}/{MAX_RECOGNITION_ATTEMPTS}<br />
            WS Status: {connectionStatus}<br />
            Connection Attempts: {connectionAttempts.current}/{MAX_CONNECTION_ATTEMPTS}<br />
            {status && (
              <>
                Backend User: {status.current_user}<br />
                Backend Attempts: {status.face_recognition_attempts}<br />
                Camera: {status.camera_active ? 'Active' : 'Inactive'}
              </>
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
    marginBottom: '1rem',
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
  infoBox: {
    fontSize: '1.1rem',
    color: '#4a5568',
    marginBottom: '2rem',
    padding: '1.5rem',
    background: 'rgba(219, 234, 254, 0.8)',
    borderRadius: '12px',
    border: '1px solid rgba(59, 130, 246, 0.3)'
  },
  warningBox: {
    fontSize: '1.1rem',
    color: '#4a5568',
    marginBottom: '2rem',
    padding: '1.5rem',
    background: 'rgba(254, 215, 170, 0.8)',
    borderRadius: '12px',
    border: '1px solid rgba(251, 146, 60, 0.4)'
  },
  successBox: {
    fontSize: '1.1rem',
    color: '#4a5568',
    marginBottom: '2rem',
    padding: '1.5rem',
    background: 'rgba(209, 250, 229, 0.8)',
    borderRadius: '12px',
    border: '1px solid rgba(52, 211, 153, 0.4)'
  },
  optionsContainer: {
    marginBottom: '2rem'
  },
  buttonGrid: {
    display: 'grid',
    gridTemplateColumns: '1fr',
    gap: '1rem'
  },
  buttonRow: {
    display: 'flex',
    gap: '0.5rem',
    justifyContent: 'center',
    flexWrap: 'wrap' as const,
    marginTop: '1.5rem'
  },
  primaryBtn: {
    padding: '1rem 2rem',
    borderRadius: '12px',
    border: 'none',
    cursor: 'pointer',
    background: 'linear-gradient(135deg, #48bb78, #38a169)',
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
  registerBtn: {
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