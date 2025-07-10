import React, { useState, useEffect } from 'react';

interface RobotStatus {
  status: string;
  message: string;
  obstacle_detected: boolean;
  obstacle_sensor?: string;  // NEW: Which sensor detected obstacle
  obstacle_distance?: number;  // NEW: Distance of obstacle
  current_speeds: Record<string, number>;
  sensor_distances?: Record<string, number>;  // NEW: Individual sensor readings
  last_command: string;
  uptime: number;
  gpio_available?: boolean;
}

type WSMessage =
  | { type: 'status_update'; data: RobotStatus }
  | { type: 'direction_executed'; data: { direction: string; success: boolean } }
  | { type: 'robot_stopped'; data: { message: string } }
  | { type: 'error'; data: { message: string } }
  | { type: 'ping' }
  | { type: 'pong' };

const ArrowKeys: React.FC = () => {
  const [ws, setWs] = useState<WebSocket | null>(null);
  const [response, setResponse] = useState<string>('');
  const [status, setStatus] = useState<RobotStatus | null>(null);
  const [connectionStatus, setConnectionStatus] = useState<'connecting' | 'connected' | 'disconnected'>('connecting');

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
            case 'direction_executed':
              setResponse(`Direction ${data.data.direction} ${data.data.success ? 'succeeded' : 'failed'}`);
              break;
            case 'robot_stopped':
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

  const sendDirection = (direction: string) => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({
        type: 'direction_command',
        data: { direction }
      }));
    } else {
      setResponse('WebSocket not connected');
    }
  };

  const sendStop = () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ type: 'stop_command' }));
    } else {
      setResponse('WebSocket not connected');
    }
  };

  const resetObstacle = () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ type: 'reset_obstacle' }));
      setResponse('Obstacle detection reset');
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
        <p style={subtitleStyle}>Use the directional buttons to control your robot</p>
        
        <div style={connectionIndicatorStyle}>
          <div style={statusDotStyle}></div>
          <span>
            {connectionStatus === 'connected' ? 'Connected' : 
             connectionStatus === 'connecting' ? 'Connecting...' : 'Disconnected'}
          </span>
        </div>

        {/* Remove the big obstacle alert banner */}

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

              {/* Remove the obstacle alert banner */}
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
            <li>Press and hold buttons to move</li>
            <li>Release to stop automatically</li>
            <li>Use STOP for emergency halt</li>
            <li>Sensors: 1=Front, 2=Left, 3=Right</li>
          </ul>
        </div>
      </div>
    </div>
  );
};

export default ArrowKeys;