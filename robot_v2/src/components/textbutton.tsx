import React, { useState, useEffect, useRef } from 'react';
import axios, { isAxiosError } from 'axios';
import { UserHeader } from './UserContext';

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
  | { type: 'text_command_processed'; data: { message: string; success: boolean } }
  | { type: 'text_command_result'; data: { message: string; success: boolean } }
  | { type: 'speech_output'; data: { text: string; success: boolean } }
  | { type: 'user_recognized'; data: { user: string | null; success: boolean } }
  | { type: 'user_registered'; data: { name: string; success: boolean } }
  | { type: 'conversation_response'; data: { mode: string; response: string } }
  | { type: 'direction_executed'; data: { direction: string; success: boolean } }
  | { type: 'robot_stopped'; data: { message: string } }
  | { type: 'obstacle_reset'; data: { message: string; success: boolean } }
  | { type: 'error'; data: { message: string } }
  | { type: 'ping' }
  | { type: 'pong' };

const TextButton: React.FC = () => {
  const [text, setText] = useState('');
  const [response, setResponse] = useState<{ message: string; isError: boolean }>({ message: '', isError: false });
  const [status, setStatus] = useState<EnhancedRobotStatus | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [retryCount, setRetryCount] = useState(0);
  const maxRetries = 5;

  // Enhanced state
  const [speechText, setSpeechText] = useState('');
  const [userRegistrationName, setUserRegistrationName] = useState('');
  const [conversationHistory, setConversationHistory] = useState<string[]>([]);
  const [showCamera, setShowCamera] = useState(false);
  const [commandPresets] = useState([
    'go forward 2 seconds',
    'turn left',
    'move backward 1 second',
    'stop',
    'move left 2 seconds then turn right',
    'Hello, how are you today?',
    'What can you do?',
    'Tell me a joke'
  ]);

  const socketRef = useRef<WebSocket | null>(null);
  const cameraRef = useRef<HTMLImageElement>(null);

  useEffect(() => {
    let pingId: number;
    let reconnectTimer: number;

    const connect = () => {
      if (retryCount >= maxRetries) {
        setResponse({ message: 'Max WS retries reached', isError: true });
        return;
      }

      const ws = new WebSocket(`ws://${window.location.hostname}:8000/ws`);
      socketRef.current = ws;

      ws.onopen = () => {
        console.log('WS connected');
        setResponse({ message: 'Connected to enhanced robot', isError: false });
        setRetryCount(0);

        pingId = window.setInterval(() => {
          if (ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ type: 'ping' }));
          }
        }, 30000);
      };

      ws.onmessage = ({ data }) => {
        let msg: WSMessage;
        try {
          msg = JSON.parse(data);
        } catch {
          setResponse({ message: 'WS parse error', isError: true });
          setIsLoading(false);
          return;
        }

        switch (msg.type) {
          case 'status_update':
            setStatus(msg.data);
            break;
          case 'text_command_processed':
          case 'text_command_result':
            setResponse({ message: msg.data.message, isError: !msg.data.success });
            addToConversationHistory(`📋 Response: ${msg.data.message}`);
            break;
          case 'speech_output':
            const speechMsg = `🤖 Robot spoke: "${msg.data.text}"`;
            setResponse({ message: speechMsg, isError: !msg.data.success });
            addToConversationHistory(speechMsg);
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
            break;
          case 'direction_executed':
            setResponse({
              message: `Direction ${msg.data.direction} ${msg.data.success ? 'succeeded' : 'failed'}`,
              isError: !msg.data.success
            });
            break;
          case 'robot_stopped':
            setResponse({ message: msg.data.message, isError: false });
            break;
          case 'obstacle_reset':
            setResponse({ message: msg.data.message, isError: !msg.data.success });
            break;
          case 'error':
            setResponse({ message: msg.data.message, isError: true });
            break;
        }

        setIsLoading(false);
      };

      ws.onerror = () => {
        console.error('WS error');
        setResponse({ message: 'WebSocket error', isError: true });
        setIsLoading(false);
      };

      ws.onclose = () => {
        console.warn('WS closed — reconnecting');
        clearInterval(pingId);
        reconnectTimer = window.setTimeout(() => {
          setRetryCount(c => c + 1);
        }, 5000);
      };
    };

    connect();
    return () => {
      clearInterval(pingId);
      clearTimeout(reconnectTimer);
      socketRef.current?.close();
    };
  }, [retryCount]);

  // Camera stream setup
  useEffect(() => {
    if (showCamera && cameraRef.current) {
      const img = cameraRef.current;
      img.src = `http://${window.location.hostname}:8000/api/camera/stream`;
    }
  }, [showCamera]);

  const addToConversationHistory = (message: string) => {
    setConversationHistory(prev => [...prev.slice(-9), message]); // Keep last 10 messages
  };

  const sendWebSocketMessage = (type: string, data: any = {}) => {
    if (socketRef.current?.readyState === WebSocket.OPEN) {
      socketRef.current.send(JSON.stringify({ type, data }));
    } else {
      setResponse({ message: 'WebSocket not connected', isError: true });
    }
  };

  const sendTextCommandViaHTTP = async (cmd: string) => {
    try {
      const res = await axios.post<{ success: boolean; message: string }>(
        `http://${window.location.hostname}:8000/api/text-command`,
        { text: cmd }
      );
      setResponse({ message: res.data.message, isError: !res.data.success });
      addToConversationHistory(`📋 HTTP Response: ${res.data.message}`);
    } catch (err) {
      let msg = 'Error sending command';
      if (isAxiosError(err)) {
        const data = err.response?.data as { detail?: string };
        msg = data.detail ?? err.message;
      }
      setResponse({ message: msg, isError: true });
    } finally {
      setIsLoading(false);
    }
  };

  const handleSubmit = () => {
    const cmd = text.trim();
    if (!cmd) {
      setResponse({ message: 'No command to send', isError: true });
      return;
    }

    setIsLoading(true);
    addToConversationHistory(`📝 You typed: "${cmd}"`);
    setText('');

    // Try WebSocket first, fallback to HTTP
    if (socketRef.current?.readyState === WebSocket.OPEN) {
      sendWebSocketMessage('text_command', { text: cmd });
    } else {
      sendTextCommandViaHTTP(cmd);
    }
  };

  const handlePresetCommand = (preset: string) => {
    setText(preset);
  };

  const sendSpeech = () => {
    if (speechText.trim()) {
      addToConversationHistory(`📢 Making robot say: "${speechText}"`);
      sendWebSocketMessage('speech_command', { text: speechText.trim() });
      setSpeechText('');
    }
  };

  const recognizeUser = () => {
    sendWebSocketMessage('recognize_user', { mode: 'text' });
  };

  const registerUser = () => {
    if (userRegistrationName.trim()) {
      sendWebSocketMessage('register_user', { name: userRegistrationName.trim() });
    }
  };

  const startConversation = () => {
    sendWebSocketMessage('conversation_mode', { mode: 'text' });
  };

  const resetObstacle = () => {
    sendWebSocketMessage('reset_obstacle');
  };

  const clearHistory = () => {
    setConversationHistory([]);
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter') {
      e.preventDefault();
      handleSubmit();
    }
  };

  return (
    <div style={styles.container}>
      <style>{`
        input, button:hover:not(:disabled) { transform: translateY(-2px); box-shadow: 0 8px 16px rgba(0,0,0,0.2); }
        input:focus, button:focus { outline: 2px solid #667eea; outline-offset: 2px; }
        button:active:not(:disabled) { transform: translateY(-4px); }
      `}</style>

      {/* User Header - Shows "Hi {name}" */}
      <div style={{ position: 'fixed', top: '1rem', left: '1rem', right: '1rem', zIndex: 1000 }}>
        <UserHeader />
      </div>

      <div style={{ ...styles.card, marginTop: '5rem' }}>
        <h2 style={styles.title}>Enhanced Text Command Control</h2>
        <p style={styles.subtitle}>Natural language commands with face recognition and speech</p>

        {/* Camera & User Management */}
        <div style={styles.section}>
          <h3 style={styles.sectionTitle}>👁️ Camera & User Recognition</h3>
          <div style={styles.buttonRow}>
            <button
              onClick={() => setShowCamera(!showCamera)}
              style={showCamera ? styles.activeBtn : styles.primaryBtn}
            >
              {showCamera ? '📷 Hide Camera' : '📷 Show Camera'}
            </button>
            <button onClick={recognizeUser} style={styles.secondaryBtn}>
              👤 Recognize User
            </button>
            <button onClick={startConversation} style={styles.secondaryBtn}>
              💬 Start Conversation
            </button>
          </div>

          {showCamera && (
            <div style={styles.cameraContainer}>
              <img
                ref={cameraRef}
                alt="Robot Camera Feed"
                style={styles.cameraFeed}
              />
            </div>
          )}

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

        {/* Speech Output Control */}
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
              disabled={!speechText.trim()}
            >
              🗣️ Speak
            </button>
          </div>
        </div>

        {/* Command Presets */}
        <div style={styles.section}>
          <h3 style={styles.sectionTitle}>🚀 Quick Commands</h3>
          <div style={styles.presetGrid}>
            {commandPresets.map((preset, index) => (
              <button
                key={index}
                onClick={() => handlePresetCommand(preset)}
                style={styles.presetBtn}
                title={`Click to use: ${preset}`}
              >
                {preset}
              </button>
            ))}
          </div>
        </div>

        {/* Main Command Input */}
        <div style={styles.section}>
          <h3 style={styles.sectionTitle}>💻 Command Input</h3>
          <label htmlFor="cmd" style={styles.label}>Enter Command</label>
          <input
            id="cmd"
            type="text"
            value={text}
            onChange={e => setText(e.target.value)}
            onKeyDown={handleKeyDown}
            placeholder="e.g. go forward 2 seconds, turn left, Hello robot!"
            style={styles.mainInput}
            disabled={isLoading}
          />

          <div style={styles.buttonRow}>
            <button
              onClick={handleSubmit}
              disabled={isLoading || !text.trim()}
              style={{
                ...styles.primaryBtn,
                background: isLoading || !text.trim() ? '#a0aec0' : 'linear-gradient(135deg, #48bb78, #38a169)',
                cursor: isLoading || !text.trim() ? 'not-allowed' : 'pointer'
              }}
            >
              {isLoading ? 'Sending…' : '📤 Send Command'}
            </button>
            
            {status?.obstacle_detected && (
              <button
                onClick={resetObstacle}
                style={styles.dangerBtn}
              >
                🚨 Reset Obstacle
              </button>
            )}
          </div>
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

        {/* Response Display */}
        {response.message && (
          <div style={{
            marginTop: '1rem',
            padding: '1rem',
            border: '2px solid',
            borderRadius: 12,
            fontSize: '0.9rem',
            textAlign: 'left' as const,
            background: response.isError ? '#fee2e2' : '#d1fae5',
            borderColor: response.isError ? '#f87171' : '#34d399'
          }}>
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
              <div><strong>Last Cmd:</strong> {status.last_command || 'None'}</div>
              <div><strong>Obstacle:</strong> <span style={{
                color: status.obstacle_detected ? '#e53e3e' : '#38a169'
              }}>{status.obstacle_detected ? 
                `Yes - ${status.obstacle_sensor}` : 'No'}</span></div>
              <div><strong>Uptime:</strong> {status.uptime?.toFixed(1)}s</div>
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
          <h4 style={styles.examplesTitle}>📋 Enhanced Text Control Guide:</h4>
          <ul style={styles.examplesList}>
            <li><strong>Movement Commands:</strong> "go forward 3 seconds", "turn left", "move backward 1 second"</li>
            <li><strong>Complex Commands:</strong> "move left 2 seconds then turn right"</li>
            <li><strong>Stop Command:</strong> "stop" (works immediately)</li>
            <li><strong>Conversations:</strong> Ask questions like "How are you?", "What can you do?"</li>
            <li><strong>Speech Output:</strong> All robot responses are spoken aloud</li>
            <li><strong>Face Recognition:</strong> Camera identifies users automatically</li>
            <li><strong>Quick Commands:</strong> Click preset buttons for common commands</li>
            <li><strong>Sensors:</strong> 1=Front, 2=Left, 3=Right obstacle detection</li>
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
    background: 'linear-gradient(135deg,#667eea,#764ba2)',
    display: 'flex' as const,
    justifyContent: 'center',
    alignItems: 'center',
    padding: '2rem'
  },
  card: {
    background: 'rgba(255,255,255,0.95)',
    borderRadius: 24,
    padding: '2rem',
    boxShadow: '0 20px 40px rgba(0,0,0,0.1)',
    maxWidth: 800,
    width: '100%',
    textAlign: 'center' as const,
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
  section: {
    marginBottom: '1.5rem',
    padding: '1rem',
    background: 'rgba(247, 250, 252, 0.7)',
    borderRadius: '12px',
    border: '1px solid rgba(226, 232, 240, 0.5)'
  },
  sectionTitle: {
    fontSize: '1.1rem',
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
  dangerBtn: {
    padding: '0.75rem 1.5rem',
    borderRadius: '8px',
    border: 'none',
    cursor: 'pointer',
    background: 'linear-gradient(135deg, #f56565, #e53e3e)',
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
  presetBtn: {
    padding: '0.5rem 1rem',
    borderRadius: '6px',
    border: '1px solid #e2e8f0',
    cursor: 'pointer',
    background: 'white',
    color: '#4a5568',
    fontSize: '0.8rem',
    fontWeight: '500',
    transition: 'all 0.3s ease',
    textAlign: 'left' as const
  },
  input: {
    flex: 1,
    padding: '0.75rem',
    borderRadius: '8px',
    border: '2px solid #e2e8f0',
    fontSize: '1rem',
    minWidth: '200px'
  },
  mainInput: {
    width: '100%',
    padding: '1rem',
    fontSize: '1rem',
    borderRadius: 12,
    border: '2px solid #e2e8f0',
    marginBottom: '1rem',
    boxSizing: 'border-box' as const,
    transition: 'all 0.3s ease'
  },
  label: {
    fontSize: '1.1rem',
    fontWeight: 600,
    marginBottom: '0.5rem',
    display: 'block',
    textAlign: 'left' as const,
    color: '#2d3748'
  },
  presetGrid: {
    display: 'grid',
    gridTemplateColumns: 'repeat(auto-fit, minmax(250px, 1fr))',
    gap: '0.5rem'
  },
  cameraContainer: {
    marginTop: '1rem',
    textAlign: 'center' as const
  },
  cameraFeed: {
    maxWidth: '100%',
    height: 'auto',
    borderRadius: '12px',
    border: '2px solid #e2e8f0'
  },
  userInfo: {
    display: 'flex',
    gap: '2rem',
    justifyContent: 'center',
    marginTop: '1rem',
    fontSize: '0.9rem',
    color: '#4a5568'
  },
  historyHeader: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: '1rem'
  },
  historyContainer: {
    maxHeight: '150px',
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
  statusCard: {
    marginTop: '1.5rem',
    padding: '1rem',
    border: '2px solid #e2e8f0',
    borderRadius: 12,
    textAlign: 'left' as const,
    background: '#f8fafc'
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
  }
};

export default TextButton;