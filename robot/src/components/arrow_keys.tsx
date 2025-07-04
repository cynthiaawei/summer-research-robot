import React, { useState, useEffect } from 'react';

interface RobotStatus {
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
}

type WSMessage =
  | { type: 'status_update'; data: RobotStatus }
  | { type: 'arrow_key_pressed'; data: { direction: string; success: boolean; message: string } }
  | { type: 'arrow_key_released'; data: { direction: string; success: boolean; message: string } }
  | { type: 'direction_executed'; data: { direction: string; success: boolean } }
  | { type: 'robot_stopped'; data: { message: string } }
  | { type: 'obstacle_reset'; data: { message: string; success: boolean } }
  | { type: 'error'; data: { message: string } }
  | { type: 'ping' }
  | { type: 'pong' };

const ArrowKeys: React.FC = () => {
  const [ws, setWs] = useState<WebSocket | null>(null);
  const [response, setResponse] = useState<string>('');
  const [status, setStatus] = useState<RobotStatus | null>(null);
  const [connectionStatus, setConnectionStatus] = useState<'connecting' | 'connected' | 'disconnected'>('connecting');
  const [pressedKeys, setPressedKeys] = useState<Set<string>>(new Set());

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
        setResponse('Connected to robot');
      };

      websocket.onmessage = (event) => {
        try {
          const data: WSMessage = JSON.parse(event.data);
          
          switch (data.type) {
            case 'status_update':
              setStatus(data.data);
              break;
            case 'arrow_key_pressed':
              setResponse(data.data.message);
              break;
            case 'arrow_key_released':
              setResponse(data.data.message);
              break;
            case 'direction_executed':
              setResponse(`Direction ${data.data.direction} ${data.data.success ? 'succeeded' : 'failed'}`);
              break;
            case 'robot_stopped':
              setResponse(data.data.message);
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
        
        // Clear pressed keys on disconnect
        setPressedKeys(new Set());
        
        // Reconnect after 3 seconds
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

  // Main function for sending arrow key commands with state tracking
  const sendArrowKeyCommand = (direction: string, keyState: 'pressed' | 'released') => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      console.log(`Sending arrow key command: ${direction} ${keyState}`);
      ws.send(JSON.stringify({
        type: 'arrow_key_command',
        data: { 
          direction,
          key_state: keyState
        }
      }));
    } else {
      setResponse('WebSocket not connected');
      console.error('WebSocket not connected');
    }
  };
  const sendStop = () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      console.log('Sending stop command');
      // Clear all pressed keys when manually stopping
      setPressedKeys(new Set());
      ws.send(JSON.stringify({ type: 'stop_command' }));
    } else {
      setResponse('WebSocket not connected');
    }
  };

  const resetObstacle = () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      console.log('Sending reset obstacle command');
      ws.send(JSON.stringify({ type: 'reset_obstacle' }));
    } else {
      setResponse('WebSocket not connected');
    }
  };

  const handleMouseEnter = (e: React.MouseEvent<HTMLButtonElement>) => {
    e.currentTarget.style.transform = 'translateY(-4px) scale(1.05)';
    e.currentTarget.style.boxShadow = '0 16px 32px rgba(102, 126, 234, 0.4)';
  };

  const handleMouseLeave = (e: React.MouseEvent<HTMLButtonElement>) => {
    e.currentTarget.style.transform = 'translateY(0) scale(1)';
    e.currentTarget.style.boxShadow = '0 8px 16px rgba(102, 126, 234, 0.3)';
  };

  // Mouse down handler with proper key state tracking
  const handleMouseDown = (e: React.MouseEvent<HTMLButtonElement>, direction: string) => {
    e.preventDefault();
    e.currentTarget.style.transform = 'translateY(-2px) scale(1.02)';
    
    // Don't send command if already pressed
    if (!pressedKeys.has(direction)) {
      console.log(`Mouse down: ${direction}`);
      setPressedKeys(prev => new Set([...prev, direction]));
      sendArrowKeyCommand(direction, 'pressed');
    }
  };

  // Mouse up handler with proper key state tracking
  const handleMouseUp = (e: React.MouseEvent<HTMLButtonElement>, direction: string) => {
    e.preventDefault();
    e.currentTarget.style.transform = 'translateY(-4px) scale(1.05)';
    
    // Only send release if key was pressed
    if (pressedKeys.has(direction)) {
      console.log(`Mouse up: ${direction}`);
      setPressedKeys(prev => {
        const newSet = new Set(prev);
        newSet.delete(direction);
        return newSet;
      });
      sendArrowKeyCommand(direction, 'released');
    }
  };

  // Mouse leave handler with proper key state tracking
  const handleMouseLeaveAndStop = (e: React.MouseEvent<HTMLButtonElement>, direction: string) => {
    handleMouseLeave(e);
    
    // Only send release if key was pressed
    if (pressedKeys.has(direction)) {
      console.log(`Mouse leave: ${direction}`);
      setPressedKeys(prev => {
        const newSet = new Set(prev);
        newSet.delete(direction);
        return newSet;
      });
      sendArrowKeyCommand(direction, 'released');
    }
  };

  // Keyboard event handlers for actual arrow keys
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      // Prevent default arrow key behavior (scrolling)
      if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.key)) {
        e.preventDefault();
      }

      const keyMap: Record<string, string> = {
        'ArrowUp': 'up',
        'ArrowDown': 'down', 
        'ArrowLeft': 'left',
        'ArrowRight': 'right'
      };

      const direction = keyMap[e.key];
      if (direction && !pressedKeys.has(direction)) {
        console.log(`Keyboard down: ${direction}`);
        setPressedKeys(prev => new Set([...prev, direction]));
        sendArrowKeyCommand(direction, 'pressed');
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      const keyMap: Record<string, string> = {
        'ArrowUp': 'up',
        'ArrowDown': 'down',
        'ArrowLeft': 'left', 
        'ArrowRight': 'right'
      };

      const direction = keyMap[e.key];
      if (direction && pressedKeys.has(direction)) {
        console.log(`Keyboard up: ${direction}`);
        setPressedKeys(prev => {
          const newSet = new Set(prev);
          newSet.delete(direction);
          return newSet;
        });
        sendArrowKeyCommand(direction, 'released');
      }
    };

    // Add event listeners
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    // Cleanup
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [pressedKeys, ws]);

  // Cleanup pressed keys when WebSocket changes
  useEffect(() => {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      setPressedKeys(new Set());
    }
  }, [ws]);

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
    padding: '3rem',
    boxShadow: '0 20px 40px rgba(0, 0, 0, 0.1)',
    border: '1px solid rgba(255, 255, 255, 0.2)',
    position: 'relative' as const
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

  return (
    <div style={containerStyle}>
      <div style={cardStyle}>
        <h2 style={titleStyle}>Arrow Key Control</h2>
        <p style={subtitleStyle}>Use the directional buttons or arrow keys to control your robot</p>
        
        <div style={connectionIndicatorStyle}>
          <div style={statusDotStyle}></div>
          <span>
            {connectionStatus === 'connected' ? 'Connected' : 
             connectionStatus === 'connecting' ? 'Connecting...' : 'Disconnected'}
          </span>
        </div>

        {/* Active keys indicator for debugging */}
        {pressedKeys.size > 0 && (
          <div style={{
            textAlign: 'center',
            marginBottom: '1rem',
            padding: '0.5rem',
            background: '#e6fffa',
            borderRadius: '8px',
            fontSize: '0.9rem',
            color: '#234e52'
          }}>
            Active: {Array.from(pressedKeys).join(', ')}
          </div>
        )}

        <div style={gridStyle}>
          <button 
            style={{
              ...buttonStyle,
              background: pressedKeys.has('up') ? 'linear-gradient(135deg, #48bb78, #38a169)' : buttonStyle.background
            }}
            onMouseDown={(e) => handleMouseDown(e, 'up')}
            onMouseUp={(e) => handleMouseUp(e, 'up')}
            onMouseLeave={(e) => handleMouseLeaveAndStop(e, 'up')}
            onMouseEnter={handleMouseEnter}
            title="Forward (Arrow Up)"
          >
            ↑
          </button>
          <div style={rowStyle}>
            <button 
              style={{
                ...buttonStyle,
                background: pressedKeys.has('left') ? 'linear-gradient(135deg, #48bb78, #38a169)' : buttonStyle.background
              }}
              onMouseDown={(e) => handleMouseDown(e, 'left')}
              onMouseUp={(e) => handleMouseUp(e, 'left')}
              onMouseLeave={(e) => handleMouseLeaveAndStop(e, 'left')}
              onMouseEnter={handleMouseEnter}
              title="Turn Left (Arrow Left)"
            >
              ←
            </button>
            <button 
              style={{
                ...buttonStyle,
                background: pressedKeys.has('right') ? 'linear-gradient(135deg, #48bb78, #38a169)' : buttonStyle.background
              }}
              onMouseDown={(e) => handleMouseDown(e, 'right')}
              onMouseUp={(e) => handleMouseUp(e, 'right')}
              onMouseLeave={(e) => handleMouseLeaveAndStop(e, 'right')}
              onMouseEnter={handleMouseEnter}
              title="Turn Right (Arrow Right)"
            >
              →
            </button>
          </div>
          <button 
            style={{
              ...buttonStyle,
              background: pressedKeys.has('down') ? 'linear-gradient(135deg, #48bb78, #38a169)' : buttonStyle.background
            }}
            onMouseDown={(e) => handleMouseDown(e, 'down')}
            onMouseUp={(e) => handleMouseUp(e, 'down')}
            onMouseLeave={(e) => handleMouseLeaveAndStop(e, 'down')}
            onMouseEnter={handleMouseEnter}
            title="Backward (Arrow Down)"
          >
            ↓
          </button>
        </div>

        <div style={{ textAlign: 'center' as const, marginBottom: '2rem' }}>
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

        <div style={{ color: '#333', textAlign: 'center' as const }}>
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
          
          {status && (
            <div style={{
              background: '#f7fafc',
              border: '1px solid #e2e8f0',
              borderRadius: '8px',
              padding: '1rem',
              textAlign: 'left' as const
            }}>
              <h3 style={{ marginTop: 0, fontSize: '1.1rem', color: '#2d3748' }}>Robot Status</h3>
              <div style={{ 
                display: 'grid', 
                gridTemplateColumns: '1fr 1fr', 
                gap: '0.5rem',
                fontSize: '0.9rem'
              }}>
                <div><strong>Status:</strong> <span style={{
                  color: status.obstacle_detected ? '#e53e3e' : 
                        status.status === 'moving' ? '#38a169' : '#718096'
                }}>{status.status}</span></div>
                <div><strong>Obstacle:</strong> <span style={{
                  color: status.obstacle_detected ? '#e53e3e' : '#38a169'
                }}>{status.obstacle_detected ? 
                  `Yes - Sensor ${status.obstacle_sensor === 'front' ? '1' : 
                                  status.obstacle_sensor === 'left' ? '2' : 
                                  status.obstacle_sensor === 'right' ? '3' : '?'}` : 'No'}</span></div>
                <div><strong>Last Command:</strong> {status.last_command || 'None'}</div>
                <div><strong>Uptime:</strong> {status.uptime?.toFixed(1)}s</div>
              </div>

              {status.gpio_available !== undefined && (
                <div style={{ marginTop: '0.5rem', fontSize: '0.8rem', color: '#718096' }}>
                  GPIO: {status.gpio_available ? 'Available' : 'Simulation Mode'}
                </div>
              )}
            </div>
          )}
        </div>

        <div style={{
          marginTop: '2rem',
          padding: '1rem',
          background: '#f0f4f8',
          borderRadius: '8px',
          fontSize: '0.9rem',
          color: '#4a5568'
        }}>
          <strong>Instructions:</strong>
          <ul style={{ margin: '0.5rem 0', paddingLeft: '1.5rem' }}>
            <li>Press and hold buttons/arrow keys to move</li>
            <li>Release to stop automatically</li>
            <li>Use STOP for emergency halt</li>
            <li>Sensors: 1=Front, 2=Left, 3=Right</li>
            <li>Active keys shown above buttons</li>
          </ul>
        </div>

        {/* Debug info - remove in production */}
        <div style={{
          marginTop: '1rem',
          padding: '0.5rem',
          background: '#f8f9fa',
          borderRadius: '4px',
          fontSize: '0.8rem',
          color: '#6c757d'
        }}>
          <strong>Debug:</strong> WebSocket State: {ws?.readyState}, Pressed Keys: {pressedKeys.size}
        </div>
      </div>
    </div>
  );
};

export default ArrowKeys;