import React, { useState, useEffect, useRef, useCallback } from 'react';
import { UserHeader } from './UserHeader';

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
}

type WSMessage =
  | { type: 'status_update'; data: EnhancedRobotStatus }
  | { type: 'speech_output'; data: { text: string; success: boolean } }
  | { type: 'speech_input'; data: { text: string | null; success: boolean } }
  | { type: 'listening_started'; data: { timeout: number } }
  | { type: 'user_recognized'; data: { user: string | null; success: boolean } }
  | { type: 'user_registered'; data: { name: string; success: boolean } }
  | { type: 'conversation_response'; data: { mode: string; response: string } }
  | { type: 'text_command_result'; data: { text: string; success: boolean; message: string } }
  | { type: 'robot_stopped'; data: { message: string } }
  | { type: 'obstacle_reset'; data: { message: string; success: boolean } }
  | { type: 'error'; data: { message: string } }
  | { type: 'ping' }
  | { type: 'pong' };

// Enhanced Camera Component
const CameraFeed: React.FC<{ showCamera: boolean; isConnected: boolean }> = ({ showCamera, isConnected }) => {
  const cameraRef = useRef<HTMLImageElement>(null);
  const [cameraStatus, setCameraStatus] = useState<'loading' | 'loaded' | 'error'>('loading');
  const [retryCount, setRetryCount] = useState(0);
  const maxRetries = 3;

  useEffect(() => {
    if (showCamera) {
      setCameraStatus('loading');
      setRetryCount(0);
    }
  }, [showCamera]);

  useEffect(() => {
    if (!showCamera || !isConnected || !cameraRef.current) {
      return;
    }

    const img = cameraRef.current;
    let timeoutId: NodeJS.Timeout;

    const handleLoad = () => {
      setCameraStatus('loaded');
      console.log('✅ Camera stream loaded successfully');
    };

    const handleError = () => {
      console.warn('❌ Camera stream failed to load');
      setCameraStatus('error');
      
      if (retryCount < maxRetries) {
        setTimeout(() => {
          if (cameraRef.current && showCamera && isConnected) {
            setRetryCount(prev => prev + 1);
            setCameraStatus('loading');
            const timestamp = Date.now();
            cameraRef.current.src = `http://${window.location.hostname}:8000/api/camera/stream?t=${timestamp}`;
          }
        }, (retryCount + 1) * 2000);
      }
    };

    timeoutId = setTimeout(() => {
      if (cameraStatus === 'loading') {
        handleError();
      }
    }, 10000);

    img.addEventListener('load', handleLoad);
    img.addEventListener('error', handleError);

    const timestamp = Date.now();
    img.src = `http://${window.location.hostname}:8000/api/camera/stream?t=${timestamp}`;

    return () => {
      if (timeoutId) clearTimeout(timeoutId);
      img.removeEventListener('load', handleLoad);
      img.removeEventListener('error', handleError);
    };
  }, [showCamera, isConnected, retryCount, cameraStatus]);

  const retryCamera = useCallback(() => {
    setRetryCount(0);
    setCameraStatus('loading');
  }, []);

  if (!showCamera) return null;

  return (
    <div style={styles.cameraContainer}>
      {cameraStatus === 'loading' && (
        <div style={styles.loadingOverlay}>
          <div style={styles.spinner}></div>
          <p>Loading camera feed...</p>
          {retryCount > 0 && <p>Retry {retryCount}/{maxRetries}</p>}
        </div>
      )}
      
      {cameraStatus === 'error' && (
        <div style={styles.errorOverlay}>
          <p>❌ Camera feed unavailable</p>
          <button onClick={retryCamera} style={styles.retryBtn}>
            🔄 Retry
          </button>
        </div>
      )}

      <img
        ref={cameraRef}
        alt="Robot Camera Feed"
        style={{
          ...styles.cameraFeed,
          opacity: cameraStatus === 'loaded' ? 1 : 0,
          transition: 'opacity 0.3s ease'
        }}
      />
    </div>
  );
};

const Speech: React.FC = () => {
  const [transcript, setTranscript] = useState('');
  const [finalTranscript, setFinalTranscript] = useState('');
  const [listening, setListening] = useState(false);
  const [recognition, setRecognition] = useState<any>(null);
  const [browserSupported, setBrowserSupported] = useState(false);
  
  const [response, setResponse] = useState<{ message: string; isError: boolean }>({ message: '', isError: false });
  const [status, setStatus] = useState<EnhancedRobotStatus | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState<'connecting' | 'connected' | 'disconnected'>('connecting');

  // Enhanced state
  const [speechText, setSpeechText] = useState('');
  const [userRegistrationName, setUserRegistrationName] = useState('');
  const [conversationHistory, setConversationHistory] = useState<string[]>([]);
  const [showCamera, setShowCamera] = useState(false);
  const [autoMode, setAutoMode] = useState(false);

  // Use refs to avoid stale closure issues
  const wsRef = useRef<WebSocket | null>(null);
  const mountedRef = useRef(true);
  const reconnectTimerRef = useRef<NodeJS.Timeout | null>(null);
  const reconnectCountRef = useRef(0);
  const maxRetries = 5;

  // Cleanup on unmount
  useEffect(() => {
    mountedRef.current = true;
    return () => {
      mountedRef.current = false;
      if (reconnectTimerRef.current) {
        clearTimeout(reconnectTimerRef.current);
        reconnectTimerRef.current = null;
      }
      if (wsRef.current) {
        wsRef.current.close();
        wsRef.current = null;
      }
    };
  }, []);

  // Initialize speech recognition
  useEffect(() => {
    if ('webkitSpeechRecognition' in window || 'SpeechRecognition' in window) {
      setBrowserSupported(true);
      const SpeechRecognition = (window as any).SpeechRecognition || (window as any).webkitSpeechRecognition;
      const recognitionInstance = new SpeechRecognition();
      
      recognitionInstance.continuous = false;
      recognitionInstance.interimResults = true;
      recognitionInstance.lang = 'en-US';

      recognitionInstance.onstart = () => {
        setListening(true);
        setTranscript('');
        setFinalTranscript('');
        setResponse({ message: 'Listening...', isError: false });
      };

      recognitionInstance.onresult = (event: any) => {
        let interimTranscript = '';
        let finalTranscriptLocal = '';

        for (let i = event.resultIndex; i < event.results.length; i++) {
          const transcript = event.results[i][0].transcript;
          if (event.results[i].isFinal) {
            finalTranscriptLocal += transcript;
          } else {
            interimTranscript += transcript;
          }
        }

        setTranscript(interimTranscript);
        if (finalTranscriptLocal) {
          setFinalTranscript(finalTranscriptLocal);
        }
      };

      recognitionInstance.onend = () => {
        setListening(false);
        if (finalTranscript) {
          handleSpeechCommand(finalTranscript);
        }
      };

      recognitionInstance.onerror = (event: any) => {
        setListening(false);
        setResponse({ message: `Speech recognition error: ${event.error}`, isError: true });
      };

      setRecognition(recognitionInstance);
    } else {
      setBrowserSupported(false);
    }
  }, [finalTranscript]);

  // Stable functions
  const addToConversationHistory = useCallback((message: string) => {
    if (!mountedRef.current) return;
    setConversationHistory(prev => [...prev.slice(-9), message]);
  }, []);

  const sendWebSocketMessage = useCallback((type: string, data: any = {}) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ type, data }));
    } else {
      setResponse({ message: 'WebSocket not connected', isError: true });
    }
  }, []);

  // Message handlers
  const handleMessage = useCallback((event: MessageEvent) => {
    if (!mountedRef.current) return;
    
    try {
      const msg: WSMessage = JSON.parse(event.data);
      
      switch (msg.type) {
        case 'status_update':
          setStatus(msg.data);
          break;
        case 'speech_output':
          const speechMsg = `🤖 Robot spoke: "${msg.data.text}"`;
          setResponse({ message: speechMsg, isError: !msg.data.success });
          addToConversationHistory(speechMsg);
          setIsLoading(false);
          break;
        case 'speech_input':
          if (msg.data.success && msg.data.text) {
            const heardMsg = `👂 Heard: "${msg.data.text}"`;
            setResponse({ message: heardMsg, isError: false });
            addToConversationHistory(heardMsg);
            handleWebSocketSpeechCommand(msg.data.text);
          } else {
            setResponse({ message: '❌ No speech detected', isError: true });
          }
          setIsLoading(false);
          break;
        case 'listening_started':
          setResponse({ message: `🎤 Listening for ${msg.data.timeout} seconds...`, isError: false });
          break;
        case 'user_recognized':
          if (msg.data.success && msg.data.user) {
            const userMsg = `👤 User recognized: ${msg.data.user}`;
            setResponse({ message: userMsg, isError: false });
            addToConversationHistory(userMsg);
          } else {
            setResponse({ message: '❓ No user recognized', isError: true });
          }
          break;
        case 'user_registered':
          const regMsg = `📝 User registration: ${msg.data.name} ${msg.data.success ? 'succeeded' : 'failed'}`;
          setResponse({ message: regMsg, isError: !msg.data.success });
          addToConversationHistory(regMsg);
          if (msg.data.success) {
            setUserRegistrationName('');
          }
          break;
        case 'conversation_response':
          const convMsg = `💬 Robot: ${msg.data.response}`;
          setResponse({ message: convMsg, isError: false });
          addToConversationHistory(convMsg);
          setIsLoading(false);
          break;
        case 'text_command_result':
          setResponse({ message: msg.data.message, isError: !msg.data.success });
          addToConversationHistory(`📋 Command: ${msg.data.message}`);
          setIsLoading(false);
          break;
        case 'robot_stopped':
          setResponse({ message: msg.data.message, isError: false });
          setIsLoading(false);
          break;
        case 'obstacle_reset':
          setResponse({ message: msg.data.message, isError: !msg.data.success });
          setIsLoading(false);
          break;
        case 'error':
          setResponse({ message: msg.data.message, isError: true });
          setIsLoading(false);
          break;
      }
    } catch (error) {
      setResponse({ message: 'WS parse error', isError: true });
      setIsLoading(false);
    }
  }, [addToConversationHistory]);

  // Connection handlers
  const handleOpen = useCallback(() => {
    if (!mountedRef.current) return;
    console.log('✅ Speech WebSocket connected');
    setConnectionStatus('connected');
    setResponse({ message: 'Connected to enhanced robot', isError: false });
    reconnectCountRef.current = 0;
  }, []);

  const handleError = useCallback(() => {
    console.error('Speech WebSocket error');
    setConnectionStatus('disconnected');
    setResponse({ message: 'WebSocket error', isError: true });
    setIsLoading(false);
  }, []);

  const handleClose = useCallback(() => {
    if (!mountedRef.current) return;
    
    console.warn('Speech WebSocket closed, retrying...');
    setConnectionStatus('disconnected');
    wsRef.current = null;
    
    if (reconnectCountRef.current < maxRetries) {
      reconnectCountRef.current++;
      reconnectTimerRef.current = setTimeout(() => {
        if (mountedRef.current) {
          initializeWebSocket();
        }
      }, 3000);
    } else {
      setResponse({ message: 'Max WS retries reached', isError: true });
    }
  }, []);

  // Initialize WebSocket
  const initializeWebSocket = useCallback(() => {
    if (!mountedRef.current) return;
    
    if (wsRef.current) {
      wsRef.current.removeEventListener('open', handleOpen);
      wsRef.current.removeEventListener('message', handleMessage);
      wsRef.current.removeEventListener('error', handleError);
      wsRef.current.removeEventListener('close', handleClose);
      wsRef.current.close();
    }
    
    setConnectionStatus('connecting');
    
    try {
      const ws = new WebSocket(`ws://${window.location.hostname}:8000/ws`);
      wsRef.current = ws;

      ws.addEventListener('open', handleOpen);
      ws.addEventListener('message', handleMessage);
      ws.addEventListener('error', handleError);
      ws.addEventListener('close', handleClose);
      
    } catch (error) {
      console.error('Failed to create WebSocket:', error);
      setConnectionStatus('disconnected');
    }
  }, [handleOpen, handleMessage, handleError, handleClose]);

  // Initialize WebSocket only once
  useEffect(() => {
    initializeWebSocket();
  }, [initializeWebSocket]);

  const handleSpeechCommand = useCallback((text: string) => {
    setIsLoading(true);
    const trimmed = text.trim();
    if (!trimmed) {
      setResponse({ message: 'Empty transcript', isError: true });
      setIsLoading(false);
      return;
    }

    addToConversationHistory(`🗣️ You said: "${trimmed}"`);
    sendWebSocketMessage('text_command', { text: trimmed.toLowerCase() });
  }, [addToConversationHistory, sendWebSocketMessage]);

  const handleWebSocketSpeechCommand = useCallback((text: string) => {
    if (text) {
      addToConversationHistory(`🗣️ You said: "${text}"`);
      sendWebSocketMessage('text_command', { text: text.toLowerCase() });
    }
  }, [addToConversationHistory, sendWebSocketMessage]);

  const startListening = useCallback(() => {
    if (recognition && !listening) {
      setTranscript('');
      setFinalTranscript('');
      recognition.start();
    }
  }, [recognition, listening]);

  const startWebSocketListening = useCallback(() => {
    setIsLoading(true);
    sendWebSocketMessage('listen_command', { timeout: 5 });
  }, [sendWebSocketMessage]);

  const stopListening = useCallback(() => {
    if (recognition && listening) {
      recognition.stop();
    }
  }, [recognition, listening]);

  const sendSpeech = useCallback(() => {
    if (speechText.trim()) {
      setIsLoading(true);
      addToConversationHistory(`📢 Making robot say: "${speechText}"`);
      sendWebSocketMessage('speech_command', { text: speechText.trim() });
      setSpeechText('');
    }
  }, [speechText, addToConversationHistory, sendWebSocketMessage]);

  const recognizeUser = useCallback(() => {
    sendWebSocketMessage('recognize_user', { mode: 'speech' });
  }, [sendWebSocketMessage]);

  const registerUser = useCallback(() => {
    if (userRegistrationName.trim()) {
      sendWebSocketMessage('register_user', { name: userRegistrationName.trim() });
    }
  }, [userRegistrationName, sendWebSocketMessage]);

  const startConversation = useCallback(() => {
    setIsLoading(true);
    sendWebSocketMessage('conversation_mode', { mode: 'speech' });
  }, [sendWebSocketMessage]);

  const toggleAutoMode = useCallback(() => {
    const newMode = !autoMode;
    setAutoMode(newMode);
    sendWebSocketMessage('set_interaction_mode', { mode: newMode ? 'auto' : 'speech' });
  }, [autoMode, sendWebSocketMessage]);

  const clearHistory = useCallback(() => {
    setConversationHistory([]);
  }, []);

  if (!browserSupported) {
    return (
      <div style={styles.container}>
        <UserHeader />
        <div style={styles.card}>
          <h2 style={styles.title}>Enhanced Speech Control</h2>
          <div style={styles.errorMessage}>
            <span style={{ color: '#e53e3e', fontSize: '1.1rem' }}>
              ❌ Your browser doesn't support speech recognition
            </span>
            <p style={{ marginTop: '1rem', color: '#718096' }}>
              Please try using Chrome, Edge, or Safari for voice control features.
            </p>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div style={styles.container}>
      <style>{`@keyframes pulse {0%{opacity:1}50%{opacity:0.5}100%{opacity:1}}
        @keyframes spin {0%{transform:rotate(0deg)}100%{transform:rotate(360deg)}}`}</style>

      <UserHeader />

      <div style={styles.card}>
        <h2 style={styles.title}>Enhanced Voice Control</h2>
        <p style={styles.subtitle}>Advanced speech interaction with face recognition</p>

        {/* Connection Status */}
        <div style={styles.connectionIndicator}>
          <div style={{
            width: '8px',
            height: '8px',
            borderRadius: '50%',
            marginRight: '8px',
            backgroundColor: connectionStatus === 'connected' ? '#48bb78' : 
                            connectionStatus === 'connecting' ? '#ed8936' : '#e53e3e'
          }}></div>
          <span>
            {connectionStatus === 'connected' ? 'Connected' : 
             connectionStatus === 'connecting' ? 'Connecting...' : 'Disconnected'}
          </span>
        </div>

        {/* Camera Controls */}
        <div style={styles.section}>
          <h3 style={styles.sectionTitle}>👁️ Camera & Recognition</h3>
          <div style={styles.buttonRow}>
            <button
              onClick={() => setShowCamera(!showCamera)}
              style={showCamera ? styles.activeBtn : styles.primaryBtn}
            >
              {showCamera ? '📷 Hide Camera' : '📷 Show Camera'}
            </button>
            <button onClick={recognizeUser} style={styles.secondaryBtn}>
              👤 Recognize Me
            </button>
            <button
              onClick={toggleAutoMode}
              style={autoMode ? styles.activeBtn : styles.secondaryBtn}
            >
              {autoMode ? '🤖 Auto Mode ON' : '🤖 Auto Mode OFF'}
            </button>
          </div>
          
          <CameraFeed showCamera={showCamera} isConnected={connectionStatus === 'connected'} />

          <div style={styles.userInfo}>
            <span><strong>Current User:</strong> {status?.current_user || 'Unknown'}</span>
            <span><strong>Hand Gesture:</strong> {status?.hand_gesture || 'none'}</span>
          </div>
        </div>

        {/* User Registration */}
        <div style={styles.section}>
          <h3 style={styles.sectionTitle}>📝 User Registration</h3>
          <div style={styles.inputRow}>
            <input
              type="text"
              placeholder="Enter your name to register"
              value={userRegistrationName}
              onChange={(e) => setUserRegistrationName(e.target.value)}
              style={styles.input}
            />
            <button onClick={registerUser} style={styles.primaryBtn}>
              Register New User
            </button>
          </div>
        </div>

        {/* Speech Controls */}
        <div style={styles.section}>
          <h3 style={styles.sectionTitle}>🎤 Speech Controls</h3>
          <div style={styles.statusContainer}>
            <span style={styles.statusText}>
              Microphone: {listening || status?.listening ? 'Listening...' : 'Ready'} 
            </span>
            <span
              style={{
                ...styles.indicator,
                background: listening || status?.listening ? '#48bb78' : '#e53e3e'
              }}
            />
          </div>

          <div style={styles.buttonRow}>
            <button
              onClick={listening ? stopListening : startListening}
              style={{
                ...styles.toggleBtn,
                background: listening ? '#f56565' : '#48bb78'
              }}
              disabled={isLoading}
            >
              {listening ? '🛑 Stop Browser Listening' : '🎤 Start Browser Listening'}
            </button>

            <button
              onClick={startWebSocketListening}
              style={styles.primaryBtn}
              disabled={isLoading}
            >
              🎤 Robot Listen
            </button>

            <button
              onClick={startConversation}
              style={styles.secondaryBtn}
              disabled={isLoading}
            >
              💬 Start Conversation
            </button>
          </div>
        </div>

        {/* Text-to-Speech */}
        <div style={styles.section}>
          <h3 style={styles.sectionTitle}>📢 Make Robot Speak</h3>
          <div style={styles.inputRow}>
            <input
              type="text"
              placeholder="Enter text for robot to speak"
              value={speechText}
              onChange={(e) => setSpeechText(e.target.value)}
              onKeyPress={(e) => e.key === 'Enter' && sendSpeech()}
              style={styles.input}
            />
            <button
              onClick={sendSpeech}
              style={styles.primaryBtn}
              disabled={isLoading || !speechText.trim()}
            >
              🗣️ Speak
            </button>
          </div>
        </div>

        {/* Transcript Display */}
        <div style={styles.transcript}>
          {transcript || finalTranscript || 'Say something to control the robot...'}
          {transcript && <span style={{ color: '#a0aec0' }}> {transcript}</span>}
        </div>

        {/* Conversation History */}
        <div style={styles.section}>
          <div style={styles.historyHeader}>
            <h3 style={styles.sectionTitle}>💬 Conversation History</h3>
            <button onClick={clearHistory} style={styles.clearBtn}>
              🗑️ Clear
            </button>
          </div>
          <div style={styles.historyContainer}>
            {conversationHistory.length === 0 ? (
              <p style={styles.emptyHistory}>No conversation yet...</p>
            ) : (
              conversationHistory.map((msg, index) => (
                <div key={index} style={styles.historyItem}>
                  {msg}
                </div>
              ))
            )}
          </div>
        </div>

        {/* Loading Indicator */}
        {isLoading && <div style={styles.loadingMessage}>Processing command…</div>}
        
        {/* Response Display */}
        {!isLoading && response.message && (
          <div
            style={{
              ...styles.response,
              background: response.isError ? '#fee2e2' : '#d1fae5',
              borderColor: response.isError ? '#f87171' : '#34d399'
            }}
          >
            <strong>{response.isError ? 'Error:' : 'Response:'}</strong> {response.message}
          </div>
        )}

        {/* Enhanced Status Display */}
        {status && (
          <div style={styles.statusCard}>
            <h3 style={styles.statusTitle}>🤖 Robot Status</h3>
            <div style={styles.statusGrid}>
              <div><strong>Status:</strong> <span style={{
                color: status.obstacle_detected ? '#e53e3e' : 
                      status.status === 'moving' ? '#38a169' : '#718096'
              }}>{status.status}</span></div>
              <div><strong>Current User:</strong> {status.current_user}</div>
              <div><strong>Hand Gesture:</strong> {status.hand_gesture}</div>
              <div><strong>Interaction Mode:</strong> {status.interaction_mode}</div>
              <div><strong>Camera:</strong> <span style={{
                color: status.camera_active ? '#38a169' : '#e53e3e'
              }}>{status.camera_active ? 'Active' : 'Inactive'}</span></div>
              <div><strong>Last Speech:</strong> {status.last_speech_output || 'None'}</div>
            </div>
            
            {status.last_speech_output && (
              <div style={styles.lastSpeech}>
                <strong>🗣️ Last Robot Speech:</strong> "{status.last_speech_output}"
              </div>
            )}

            <div style={styles.featureStatus}>
              <div><strong>Available Features:</strong></div>
              <div>Face Recognition: {status.face_recognition_available ? '✅' : '❌'}</div>
              <div>Speech Recognition: {status.speech_recognition_available ? '✅' : '❌'}</div>
              <div>Hand Detection: {status.mediapipe_available ? '✅' : '❌'}</div>
              <div>Camera: {status.camera_active ? '✅' : '❌'}</div>
              <div>GPIO: {status.gpio_available ? '✅' : '❌'}</div>
            </div>
          </div>
        )}

        {/* Instructions */}
        <div style={styles.examples}>
          <h4 style={styles.examplesTitle}>📋 How to Use Enhanced Speech Control:</h4>
          <ul style={styles.examplesList}>
            <li><strong>Face Recognition:</strong> Click "Show Camera" and "Recognize Me" to identify yourself</li>
            <li><strong>Auto Mode:</strong> Enable for automatic responses to gestures and face recognition</li>
            <li><strong>Browser Listening:</strong> Uses your browser's speech recognition (works offline)</li>
            <li><strong>Robot Listen:</strong> Uses robot's advanced speech recognition (requires internet)</li>
            <li><strong>Voice Commands:</strong> "Go forward", "Turn left", "Move backward 2 seconds", "Stop"</li>
            <li><strong>Conversations:</strong> Ask questions like "How are you?" or give complex commands</li>
            <li><strong>Speech Output:</strong> All robot responses are spoken aloud and shown here</li>
          </ul>
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
    padding: '2rem',
    paddingTop: '6rem'
  },
  card: {
    background: 'rgba(255,255,255,0.95)',
    borderRadius: '24px',
    padding: '2rem',
    boxShadow: '0 20px 40px rgba(0,0,0,0.1)',
    textAlign: 'center' as const,
    maxWidth: '800px',
    width: '100%',
    maxHeight: '90vh',
    overflowY: 'auto' as const
  },
  title: {
    fontSize: '2rem',
    fontWeight: '700',
    color: '#2d3748',
    marginBottom: '0.5rem'
  },
  subtitle: {
    color: '#718096',
    marginBottom: '2rem',
    fontSize: '1.1rem'
  },
  connectionIndicator: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    marginBottom: '1rem',
    fontSize: '0.9rem'
  },
  section: {
    marginBottom: '2rem',
    padding: '1rem',
    background: 'rgba(247, 250, 252, 0.7)',
    borderRadius: '12px',
    border: '1px solid rgba(226, 232, 240, 0.5)'
  },
  sectionTitle: {
    fontSize: '1.2rem',
    fontWeight: '600',
    color: '#2d3748',
    marginBottom: '1rem',
    marginTop: 0
  },
  buttonRow: {
    display: 'flex',
    gap: '0.5rem',
    justifyContent: 'center',
    flexWrap: 'wrap' as const,
    marginBottom: '1rem'
  },
  inputRow: {
    display: 'flex',
    gap: '0.5rem',
    alignItems: 'center',
    flexWrap: 'wrap' as const
  },
  primaryBtn: {
    padding: '0.75rem 1.5rem',
    borderRadius: '8px',
    border: 'none',
    cursor: 'pointer',
    background: 'linear-gradient(135deg, #667eea, #764ba2)',
    color: 'white',
    fontSize: '0.9rem',
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
  activeBtn: {
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
  toggleBtn: {
    padding: '0.75rem 1.5rem',
    borderRadius: '8px',
    border: 'none',
    cursor: 'pointer',
    color: 'white',
    fontSize: '0.9rem',
    fontWeight: '600',
    transition: 'all 0.3s ease'
  },
  clearBtn: {
    padding: '0.5rem 1rem',
    borderRadius: '6px',
    border: 'none',
    cursor: 'pointer',
    background: '#e53e3e',
    color: 'white',
    fontSize: '0.8rem',
    fontWeight: '600'
  },
  retryBtn: {
    padding: '0.5rem 1rem',
    borderRadius: '6px',
    border: 'none',
    cursor: 'pointer',
    background: '#667eea',
    color: 'white',
    fontSize: '0.9rem',
    fontWeight: '600',
    marginTop: '0.5rem'
  },
  input: {
    flex: 1,
    padding: '0.75rem',
    borderRadius: '8px',
    border: '2px solid #e2e8f0',
    fontSize: '1rem'
  },
  cameraContainer: {
    position: 'relative' as const,
    textAlign: 'center' as const,
    marginTop: '1rem',
    minHeight: '300px',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center'
  },
  cameraFeed: {
    maxWidth: '100%',
    height: 'auto',
    borderRadius: '12px',
    border: '2px solid #e2e8f0',
    minHeight: '300px',
    backgroundColor: '#f7fafc'
  },
  loadingOverlay: {
    position: 'absolute' as const,
    top: '50%',
    left: '50%',
    transform: 'translate(-50%, -50%)',
    textAlign: 'center' as const,
    color: '#4a5568',
    zIndex: 10
  },
  errorOverlay: {
    position: 'absolute' as const,
    top: '50%',
    left: '50%',
    transform: 'translate(-50%, -50%)',
    textAlign: 'center' as const,
    color: '#e53e3e',
    zIndex: 10
  },
  spinner: {
    width: '40px',
    height: '40px',
    border: '4px solid #f3f3f3',
    borderTop: '4px solid #667eea',
    borderRadius: '50%',
    animation: 'spin 1s linear infinite',
    margin: '0 auto 1rem'
  },
  userInfo: {
    display: 'flex',
    gap: '2rem',
    justifyContent: 'center',
    marginTop: '1rem',
    fontSize: '0.9rem',
    color: '#4a5568'
  },
  statusContainer: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    marginBottom: '1rem'
  },
  statusText: { 
    fontSize: '1rem',
    marginRight: '0.5rem',
    color: '#4a5568'
  },
  indicator: {
    display: 'inline-block',
    width: 12,
    height: 12,
    borderRadius: '50%',
    animation: 'pulse 2s infinite'
  },
  transcript: {
    background: '#f7fafc',
    borderRadius: 12,
    padding: '1.5rem',
    minHeight: 60,
    border: '2px solid #e2e8f0',
    margin: '1rem 0',
    fontSize: '1rem',
    lineHeight: '1.5',
    color: '#2d3748',
    textAlign: 'left' as const
  },
  historyHeader: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: '1rem'
  },
  historyContainer: {
    maxHeight: '200px',
    overflowY: 'auto' as const,
    background: '#f8fafc',
    borderRadius: '8px',
    padding: '1rem',
    textAlign: 'left' as const
  },
  historyItem: {
    padding: '0.5rem 0',
    borderBottom: '1px solid #e2e8f0',
    fontSize: '0.9rem',
    lineHeight: '1.4'
  },
  emptyHistory: {
    textAlign: 'center' as const,
    color: '#a0aec0',
    fontStyle: 'italic'
  },
  loadingMessage: {
    color: '#4a5568',
    fontStyle: 'italic',
    marginBottom: '1rem',
    padding: '1rem',
    background: '#f0f4f8',
    borderRadius: '8px'
  },
  response: {
    borderRadius: 12,
    padding: '1rem',
    border: '2px solid',
    marginBottom: '1rem',
    textAlign: 'left' as const
  },
  statusCard: {
    background: '#f8fafc',
    borderRadius: 12,
    padding: '1rem',
    border: '2px solid #e2e8f0',
    textAlign: 'left' as const,
    marginTop: '1rem'
  },
  statusTitle: {
    marginTop: 0,
    marginBottom: '1rem',
    fontSize: '1.1rem',
    color: '#2d3748'
  },
  statusGrid: {
    display: 'grid',
    gridTemplateColumns: '1fr 1fr',
    gap: '0.5rem',
    fontSize: '0.9rem',
    marginBottom: '1rem'
  },
  lastSpeech: {
    background: '#e6fffa',
    padding: '0.75rem',
    borderRadius: '8px',
    marginBottom: '1rem',
    fontSize: '0.9rem',
    border: '1px solid #38b2ac'
  },
  featureStatus: {
    fontSize: '0.8rem',
    color: '#718096',
    background: '#f0f4f8',
    padding: '0.75rem',
    borderRadius: '6px'
  },
  examples: {
    marginTop: '2rem',
    padding: '1rem',
    background: '#f0f4f8',
    borderRadius: 12,
    textAlign: 'left' as const
  },
  examplesTitle: {
    marginTop: 0,
    marginBottom: '0.5rem',
    fontSize: '1rem',
    color: '#4a5568'
  },
  examplesList: {
    margin: 0,
    paddingLeft: '1.5rem',
    fontSize: '0.9rem',
    color: '#718096'
  },
  errorMessage: {
    textAlign: 'center' as const,
    padding: '2rem'
  }
};

export default Speech;