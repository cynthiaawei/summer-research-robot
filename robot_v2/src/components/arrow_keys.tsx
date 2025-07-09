import React, { useState, useEffect, useRef } from 'react';

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
  | { type: 'direction_executed'; data: { direction: string; success: boolean } }
  | { type: 'robot_stopped'; data: { message: string } }
  | { type: 'speech_output'; data: { text: string; success: boolean } }
  | { type: 'user_recognized'; data: { user: string | null; success: boolean } }
  | { type: 'user_registered'; data: { name: string; success: boolean } }
  | { type: 'obstacle_reset'; data: { message: string; success: boolean } }
  | { type: 'error'; data: { message: string } }
  | { type: 'ping' }
  | { type: 'pong' };

const ArrowKeys: React.FC = () => {
  const [ws, setWs] = useState<WebSocket | null>(null);
  const [response, setResponse] = useState<string>('');
  const [status, setStatus] = useState<EnhancedRobotStatus | null>(null);
  const [connectionStatus, setConnectionStatus] = useState<'connecting' | 'connected' | 'disconnected'>('connecting');
  
  // Enhanced state
  const [showCamera, setShowCamera] = useState(false);
  const [userRegistrationName, setUserRegistrationName] = useState('');
  const [speechText, setSpeechText] = useState('');
  const [autoMode, setAutoMode] = useState(false);
  const [movementHistory, setMovementHistory] = useState<string[]>([]);

  const cameraRef = useRef<HTMLImageElement>(null);

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
        setResponse('Connected to enhanced robot');
      };

      websocket.onmessage = (event) => {
        try {
          const data: WSMessage = JSON.parse(event.data);
          
          switch (data.type) {
            case 'status_update':
              setStatus(data.data);
              break;
            case 'direction_executed':
              const dirMsg = `Direction ${data.data.direction} ${data.data.success ? 'succeeded' : 'failed'}`;
              setResponse(dirMsg);
              addToMovementHistory(dirMsg);
              break;
            case 'robot_stopped':
              setResponse(data.data.message);
              addToMovementHistory('🛑 Robot stopped');
              break;
            case 'speech_output':
              setResponse(`🤖 Robot spoke: "${data.data.text}"`);
              break;
            case 'user_recognized':
              if (data.data.success && data.data.user) {
                setResponse(`👤 User recognized: ${data.data.user}`);
              } else {
                setResponse('❓ No user recognized');
              }
              break;
            case 'user_registered':
              setResponse(`📝 User registration: ${data.data.name} ${data.data.success ? 'succeeded' : 'failed'}`);
              if (data.data.success) {
                setUserRegistrationName('');
              }
              break;
            case 'obstacle_reset':
              setResponse(data.data.message);
              break;
            case 'error':
              setResponse(`Error: ${data.data.message}`);
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
        console.log('WebSocket closed');
        setConnectionStatus('disconnected');
        setResponse('WebSocket disconnected - reconnecting...');
        setWs(null);
        
        reconnectTimer = setTimeout(connect, 3000);
      };
    };

    connect();

    return () => {
      if (reconnectTimer) {
        clearTimeout(reconnectTimer);
      }
      if (ws) {
        ws.close();
      }
    };
  }, []);

  // Camera stream setup
  useEffect(() => {
    if (showCamera && cameraRef.current) {
      const img = cameraRef.current;
      img.src = `http://${window.location.hostname}:8000/api/camera/stream`;
    }
  }, [showCamera]);

  const addToMovementHistory = (message: string) => {
    setMovementHistory(prev => [...prev.slice(-4), message]); // Keep last 5 movements
  };

  const sendWebSocketMessage = (type: string, data: any = {}) => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ type, data }));
    } else {
      setResponse('WebSocket not connected');
    }
  };

  const sendDirection = (direction: string) => {
    sendWebSocketMessage('direction_command', { direction });
    addToMovementHistory(`▶️ Command sent: ${direction}`);
  };

  const sendStop = () => {
    sendWebSocketMessage('stop_command');
    addToMovementHistory('🛑 Stop command sent');
  };

  const sendSpeech = () => {
    if (speechText.trim()) {
      sendWebSocketMessage('speech_command', { text: speechText.trim() });
      setSpeechText('');
    }
  };

  const recognizeUser = () => {
    sendWebSocketMessage('recognize_user', { mode: 'auto' });
  };

  const registerUser = () => {
    if (userRegistrationName.trim()) {
      sendWebSocketMessage('register_user', { name: userRegistrationName.trim() });
    }
  };

  const toggleAutoMode = () => {
    const newMode = !autoMode;
    setAutoMode(newMode);
    sendWebSocketMessage('set_interaction_mode', { mode: newMode ? 'auto' : 'manual' });
  };

  const resetObstacle = () => {
    sendWebSocketMessage('reset_obstacle');
    setResponse('Obstacle reset requested...');
  };

  const clearHistory = () => {
    setMovementHistory([]);
  };

  const handleMouseEnter = (e: React.MouseEvent<HTMLButtonElement>) => {
    e.currentTarget.style.transform = 'translateY(-4px) scale(1.05)';
    e.currentTarget.style.boxShadow = '0 16px 32px rgba(102, 126, 234, 0.4)';
  };

  const handleMouseLeave = (e: React.MouseEvent<HTMLButtonElement>) => {
    e.currentTarget.style.transform = 'translateY(0) scale(1)';
    e.currentTarget.style.boxShadow = '0 8px 16px rgba(102, 126, 234, 0.3)';
  };

  const handleMouseDown = (e: React.MouseEvent<HTMLButtonElement>, direction: string) => {
    e.currentTarget.style.transform = 'translateY(-2px) scale(1.02)';
    sendDirection(direction);
  };

  const handleMouseUp = (e: React.MouseEvent<HTMLButtonElement>) => {
    e.currentTarget.style.transform = 'translateY(-4px) scale(1.05)';
    sendStop();
  };

  const handleMouseLeaveAndStop = (e: React.MouseEvent<HTMLButtonElement>) => {
    handleMouseLeave(e);
    sendStop();
  };

  const containerStyle = {
    minHeight: '100vh',
    width: '100vw',
    background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
    display: 'flex',
    justifyContent: 'center',
    alignItems: 'center',
    fontFamily: '"Inter", sans-serif',
    padding: '2rem'
  };

  const cardStyle = {
    background: 'rgba(255, 255, 255, 0.95)',
    backdropFilter: 'blur(20px)',
    borderRadius: '24px',
    padding: '2rem',
    boxShadow: '0 20px 40px rgba(0, 0, 0, 0.1)',
    border: '1px solid rgba(255, 255, 255, 0.2)',
    position: 'relative' as const,
    maxWidth: '900px',
    width: '100%'
  };

  const titleStyle = {
    fontSize: '2rem',
    fontWeight: '700',
    color: '#2d3748',
    marginBottom: '0.5rem',
    textAlign: 'center' as const
  };

  const subtitleStyle = {
    color: '#718096',
    marginBottom: '2rem',
    textAlign: 'center' as const,
    fontSize: '1.1rem'
  };

  const connectionIndicatorStyle = {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    marginBottom: '1rem',
    fontSize: '0.9rem'
  };

  const statusDotStyle = {
    width: '8px',
    height: '8px',
    borderRadius: '50%',
    marginRight: '8px',
    backgroundColor: connectionStatus === 'connected' ? '#48bb78' : 
                    connectionStatus === 'connecting' ? '#ed8936' : '#e53e3e'
  };

  const sectionStyle = {
    marginBottom: '2rem',
    padding: '1rem',
    background: 'rgba(247, 250, 252, 0.7)',
    borderRadius: '12px',
    border: '1px solid rgba(226, 232, 240, 0.5)'
  };

  const sectionTitleStyle = {
    fontSize: '1.2rem',
    fontWeight: '600',
    color: '#2d3748',
    marginBottom: '1rem',
    marginTop: 0
  };

  const buttonRowStyle = {
    display: 'flex',
    gap: '0.5rem',
    justifyContent: 'center',
    flexWrap: 'wrap' as const,
    marginBottom: '1rem'
  };

  const primaryBtnStyle = {
    padding: '0.75rem 1.5rem',
    borderRadius: '8px',
    border: 'none',
    cursor: 'pointer',
    background: 'linear-gradient(135deg, #667eea, #764ba2)',
    color: 'white',
    fontSize: '0.9rem',
    fontWeight: '600',
    transition: 'all 0.3s ease'
  };

  const secondaryBtnStyle = {
    padding: '0.75rem 1.5rem',
    borderRadius: '8px',
    border: 'none',
    cursor: 'pointer',
    background: 'linear-gradient(135deg, #48bb78, #38a169)',
    color: 'white',
    fontSize: '0.9rem',
    fontWeight: '600',
    transition: 'all 0.3s ease'
  };

  const activeBtnStyle = {
    padding: '0.75rem 1.5rem',
    borderRadius: '8px',
    border: 'none',
    cursor: 'pointer',
    background: 'linear-gradient(135deg, #ed8936, #dd6b20)',
    color: 'white',
    fontSize: '0.9rem',
    fontWeight: '600',
    transition: 'all 0.3s ease'
  };

  const dangerBtnStyle = {
    padding: '0.75rem 1.5rem',
    borderRadius: '8px',
    border: 'none',
    cursor: 'pointer',
    background: 'linear-gradient(135deg, #f56565, #e53e3e)',
    color: 'white',
    fontSize: '0.9rem',
    fontWeight: '600',
    transition: 'all 0.3s ease'
  };

  const gridStyle = {
    display: 'flex',
    flexDirection: 'column' as const,
    alignItems: 'center',
    gap: '1rem',
    marginBottom: '2rem'
  };

  const rowStyle = {
    display: 'flex',
    gap: '1rem'
  };

  const buttonStyle = {
    width: '80px',
    height: '80px',
    border: 'none',
    borderRadius: '20px',
    background: 'linear-gradient(135deg, #667eea, #764ba2)',
    color: 'white',
    fontSize: '1.5rem',
    fontWeight: '600',
    cursor: 'pointer',
    transition: 'all 0.3s cubic-bezier(0.4, 0, 0.2, 1)',
    boxShadow: '0 8px 16px rgba(102, 126, 234, 0.3)',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    userSelect: 'none' as const
  };

  const stopButtonStyle = {
    ...buttonStyle,
    width: '120px',
    height: '50px',
    background: 'linear-gradient(135deg, #f56565, #e53e3e)',
    fontSize: '1rem'
  };

  const inputStyle = {
    flex: 1,
    padding: '0.75rem',
    borderRadius: '8px',
    border: '2px solid #e2e8f0',
    fontSize: '1rem',
    minWidth: '200px'
  };

  return (
    <div style={containerStyle}>
      <div style={cardStyle}>
        <h2 style={titleStyle}>Enhanced Arrow Key Control</h2>
        <p style={subtitleStyle}>Direct robot control with face recognition and speech</p>
        
        <div style={connectionIndicatorStyle}>
          <div style={statusDotStyle}></div>
          <span>
            {connectionStatus === 'connected' ? 'Connected' : 
             connectionStatus === 'connecting' ? 'Connecting...' : 'Disconnected'}
          </span>
        </div>

        {/* Camera & User Management */}
        <div style={sectionStyle}>
          <h3 style={sectionTitleStyle}>👁️ Camera & Recognition</h3>
          <div style={buttonRowStyle}>
            <button
              onClick={() => setShowCamera(!showCamera)}
              style={showCamera ? activeBtnStyle : primaryBtnStyle}
            >
              {showCamera ? '📷 Hide Camera' : '📷 Show Camera'}
            </button>
            <button onClick={recognizeUser} style={secondaryBtnStyle}>
              👤 Recognize User
            </button>
            <button
              onClick={toggleAutoMode}
              style={autoMode ? activeBtnStyle : secondaryBtnStyle}
            >
              {autoMode ? '🤖 Auto Mode ON' : '🤖 Auto Mode OFF'}
            </button>
          </div>

          {showCamera && (
            <div style={{ textAlign: 'center', marginTop: '1rem' }}>
              <img
                ref={cameraRef}
                alt="Robot Camera Feed"
                style={{
                  maxWidth: '100%',
                  height: 'auto',
                  borderRadius: '12px',
                  border: '2px solid #e2e8f0'
                }}
              />
            </div>
          )}

          <div style={{ 
            display: 'flex', 
            gap: '2rem', 
            justifyContent: 'center', 
            marginTop: '1rem',
            fontSize: '0.9rem',
            color: '#4a5568'
          }}>
            <span><strong>Current User:</strong> {status?.current_user || 'Unknown'}</span>
            <span><strong>Hand Gesture:</strong> {status?.hand_gesture || 'none'}</span>
          </div>
        </div>

        {/* User Registration */}
        <div style={sectionStyle}>
          <h3 style={sectionTitleStyle}>📝 User Registration</h3>
          <div style={{ display: 'flex', gap: '0.5rem', alignItems: 'center', flexWrap: 'wrap' }}>
            <input
              type="text"
              placeholder="Enter your name to register"
              value={userRegistrationName}
              onChange={(e) => setUserRegistrationName(e.target.value)}
              style={inputStyle}
            />
            <button onClick={registerUser} style={primaryBtnStyle}>
              Register New User
            </button>
          </div>
        </div>

        {/* Speech Controls */}
        <div style={sectionStyle}>
          <h3 style={sectionTitleStyle}>📢 Speech Control</h3>
          <div style={{ display: 'flex', gap: '0.5rem', alignItems: 'center', flexWrap: 'wrap' }}>
            <input
              type="text"
              placeholder="Enter text for robot to speak"
              value={speechText}
              onChange={(e) => setSpeechText(e.target.value)}
              onKeyPress={(e) => e.key === 'Enter' && sendSpeech()}
              style={inputStyle}
            />
            <button
              onClick={sendSpeech}
              style={primaryBtnStyle}
              disabled={!speechText.trim()}
            >
              🗣️ Speak
            </button>
          </div>
          
          {status?.last_speech_output && (
            <div style={{
              marginTop: '1rem',
              background: '#e6fffa',
              padding: '0.75rem',
              borderRadius: '8px',
              fontSize: '0.9rem',
              border: '1px solid #38b2ac'
            }}>
              <strong>🗣️ Last Robot Speech:</strong> "{status.last_speech_output}"
            </div>
          )}
        </div>

        {/* Movement Controls */}
        <div style={sectionStyle}>
          <h3 style={sectionTitleStyle}>🎮 Movement Controls</h3>
          <div style={gridStyle}>
            <button 
              style={buttonStyle}
              onMouseDown={(e) => handleMouseDown(e, 'up')}
              onMouseUp={handleMouseUp}
              onMouseLeave={handleMouseLeaveAndStop}
              onMouseEnter={handleMouseEnter}
              title="Forward"
            >
              ↑
            </button>
            <div style={rowStyle}>
              <button 
                style={buttonStyle}
                onMouseDown={(e) => handleMouseDown(e, 'left')}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseLeaveAndStop}
                onMouseEnter={handleMouseEnter}
                title="Turn Left"
              >
                ←
              </button>
              <button 
                style={buttonStyle}
                onMouseDown={(e) => handleMouseDown(e, 'right')}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseLeaveAndStop}
                onMouseEnter={handleMouseEnter}
                title="Turn Right"
              >
                →
              </button>
            </div>
            <button 
              style={buttonStyle}
              onMouseDown={(e) => handleMouseDown(e, 'down')}
              onMouseUp={handleMouseUp}
              onMouseLeave={handleMouseLeaveAndStop}
              onMouseEnter={handleMouseEnter}
              title="Backward"
            >
              ↓
            </button>
          </div>

          <div style={{ textAlign: 'center', marginBottom: '1rem' }}>
            <button
              onClick={sendStop}
              style={stopButtonStyle}
              onMouseEnter={(e) => {
                e.currentTarget.style.transform = 'translateY(-2px)';
                e.currentTarget.style.boxShadow = '0 12px 24px rgba(245, 101, 101, 0.4)';
              }}
              onMouseLeave={(e) => {
                e.currentTarget.style.transform = 'translateY(0)';
                e.currentTarget.style.boxShadow = '0 8px 16px rgba(102, 126, 234, 0.3)';
              }}
              title="Emergency Stop"
            >
              STOP
            </button>
            
            {status?.obstacle_detected && (
              <button
                onClick={resetObstacle}
                style={{
                  ...stopButtonStyle,
                  background: 'linear-gradient(135deg, #ed8936, #dd6b20)',
                  marginLeft: '1rem'
                }}
                onMouseEnter={(e) => {
                  e.currentTarget.style.transform = 'translateY(-2px)';
                  e.currentTarget.style.boxShadow = '0 12px 24px rgba(237, 137, 54, 0.4)';
                }}
                onMouseLeave={(e) => {
                  e.currentTarget.style.transform = 'translateY(0)';
                  e.currentTarget.style.boxShadow = '0 8px 16px rgba(102, 126, 234, 0.3)';
                }}
                title="Reset Obstacle Detection"
              >
                RESET
              </button>
            )}
          </div>
        </div>

        {/* Movement History */}
        <div style={sectionStyle}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '1rem' }}>
            <h3 style={sectionTitleStyle}>📜 Movement History</h3>
            <button
              onClick={clearHistory}
              style={{
                padding: '0.5rem 1rem',
                borderRadius: '6px',
                border: 'none',
                cursor: 'pointer',
                background: '#e53e3e',
                color: 'white',
                fontSize: '0.8rem',
                fontWeight: '600'
              }}
            >
              🗑️ Clear
            </button>
          </div>
          <div style={{
            maxHeight: '120px',
            overflowY: 'auto',
            background: '#f8fafc',
            borderRadius: '8px',
            padding: '1rem'
          }}>
            {movementHistory.length === 0 ? (
              <p style={{ textAlign: 'center', color: '#a0aec0', fontStyle: 'italic' }}>
                No movement commands yet...
              </p>
            ) : (
              movementHistory.map((move, index) => (
                <div key={index} style={{
                  padding: '0.5rem 0',
                  borderBottom: index < movementHistory.length - 1 ? '1px solid #e2e8f0' : 'none',
                  fontSize: '0.9rem'
                }}>
                  {move}
                </div>
              ))
            )}
          </div>
        </div>

        {/* Response Display */}
        <div style={{ color: '#333', textAlign: 'center' }}>
          {response && (
            <div style={{
              background: response.includes('Error') ? '#fee2e2' : '#d1fae5',
              border: `1px solid ${response.includes('Error') ? '#f87171' : '#68d391'}`,
              borderRadius: '8px',
              padding: '1rem',
              marginBottom: '1rem'
            }}>
              <strong>Response:</strong> {response}
            </div>
          )}
          
          {/* Enhanced Status Display */}
          {status && (
            <div style={{
              background: '#f7fafc',
              border: '1px solid #e2e8f0',
              borderRadius: '8px',
              padding: '1rem',
              textAlign: 'left' as const
            }}>
              <h3 style={{ marginTop: 0, fontSize: '1.1rem', color: '#2d3748' }}>🤖 Robot Status</h3>
              <div style={{ 
                display: 'grid', 
                gridTemplateColumns: '1fr 1fr 1fr', 
                gap: '0.5rem',
                fontSize: '0.9rem',
                marginBottom: '1rem'
              }}>
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
                <div><strong>Obstacle:</strong> <span style={{
                  color: status.obstacle_detected ? '#e53e3e' : '#38a169'
                }}>{status.obstacle_detected ? 
                  `Yes - Sensor ${status.obstacle_sensor === 'front' ? '1' : 
                                  status.obstacle_sensor === 'left' ? '2' : 
                                  status.obstacle_sensor === 'right' ? '3' : '?'}` : 'No'}</span></div>
                <div><strong>Last Command:</strong> {status.last_command || 'None'}</div>
                <div><strong>Uptime:</strong> {status.uptime?.toFixed(1)}s</div>
                <div><strong>GPIO:</strong> {status.gpio_available ? '✅' : '❌'}</div>
              </div>

              <div style={{ fontSize: '0.8rem', color: '#718096', background: '#f0f4f8', padding: '0.75rem', borderRadius: '6px' }}>
                <div><strong>Available Features:</strong></div>
                <div>Face Recognition: {status.face_recognition_available ? '✅' : '❌'}</div>
                <div>Speech Recognition: {status.speech_recognition_available ? '✅' : '❌'}</div>
                <div>Hand Detection: {status.mediapipe_available ? '✅' : '❌'}</div>
                <div>Camera: {status.camera_active ? '✅' : '❌'}</div>
              </div>
            </div>
          )}
        </div>

        {/* Instructions */}
        <div style={{
          marginTop: '2rem',
          padding: '1rem',
          background: '#f0f4f8',
          borderRadius: '8px',
          fontSize: '0.9rem',
          color: '#4a5568'
        }}>
          <strong>📋 Enhanced Arrow Key Instructions:</strong>
          <ul style={{ margin: '0.5rem 0', paddingLeft: '1.5rem' }}>
            <li><strong>Movement:</strong> Press and hold arrow buttons to move, release to stop automatically</li>
            <li><strong>Emergency:</strong> Use STOP button for immediate halt</li>
            <li><strong>Face Recognition:</strong> Camera identifies users automatically in Auto Mode</li>
            <li><strong>Speech:</strong> Enter text and click "Speak" to make robot talk</li>
            <li><strong>Auto Mode:</strong> Robot responds to hand gestures and face recognition</li>
            <li><strong>Sensors:</strong> 1=Front, 2=Left, 3=Right obstacle detection</li>
            <li><strong>History:</strong> Track recent movement commands</li>
          </ul>
        </div>
      </div>
    </div>
  );
};

export default ArrowKeys;