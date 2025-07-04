import React, { useState, useEffect } from 'react';

const ArrowKeys: React.FC = () => {
  const [ws, setWs] = useState<WebSocket | null>(null);
  const [response, setResponse] = useState<string>('');
  const [status, setStatus] = useState<any>(null);

  // Setup WebSocket connection
  useEffect(() => {
    const websocket = new WebSocket(`ws://${window.location.hostname}:8000/ws`);
    setWs(websocket);

    websocket.onopen = () => {
      console.log('WebSocket connected');
    };

    websocket.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.type === 'status_update') {
        setStatus(data.data);
      } else if (data.type === 'direction_executed') {
        setResponse(`Direction ${data.data.direction} ${data.data.success ? 'succeeded' : 'failed'}`);
      } else if (data.type === 'robot_stopped') {
        setResponse(data.data.message);
      } else if (data.type === 'error') {
        setResponse(data.data.message);
      }
    };

    websocket.onerror = () => {
      console.error('WebSocket error');
      setResponse('WebSocket connection error');
    };

    websocket.onclose = () => {
      console.log('WebSocket closed');
      setResponse('WebSocket disconnected');
      setWs(null);
    };

    return () => websocket.close();
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
    border: '1px solid rgba(255, 255, 255, 0.2)'
  };

  const gridStyle = {
    display: 'flex',
    flexDirection: 'column' as const,
    alignItems: 'center',
    gap: '1rem'
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
    justifyContent: 'center'
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

  return (
    <div style={containerStyle}>
      <div style={cardStyle}>
        <div style={gridStyle}>
          <button 
            style={buttonStyle}
            onMouseDown={(e) => handleMouseDown(e, 'up')}
            onMouseUp={handleMouseUp}
            onMouseEnter={handleMouseEnter}
            onMouseLeave={handleMouseLeave}
          >
            ↑
          </button>
          <div style={rowStyle}>
            <button 
              style={buttonStyle}
              onMouseDown={(e) => handleMouseDown(e, 'left')}
              onMouseUp={handleMouseUp}
              onMouseEnter={handleMouseEnter}
              onMouseLeave={handleMouseLeave}
            >
              ←
            </button>
            <button 
              style={buttonStyle}
              onMouseDown={(e) => handleMouseDown(e, 'right')}
              onMouseUp={handleMouseUp}
              onMouseEnter={handleMouseEnter}
              onMouseLeave={handleMouseLeave}
            >
              →
            </button>
          </div>
          <button 
            style={buttonStyle}
            onMouseDown={(e) => handleMouseDown(e, 'down')}
            onMouseUp={handleMouseUp}
            onMouseEnter={handleMouseEnter}
            onMouseLeave={handleMouseLeave}
          >
            ↓
          </button>
        </div>
        <div style={{ marginTop: '2rem', color: '#333' }}>
          <p><strong>Response:</strong> {response}</p>
          {status && (
            <div>
              <h3>Robot Status</h3>
              <p><strong>Status:</strong> {status.status}</p>
              <p><strong>Message:</strong> {status.message}</p>
              <p><strong>Obstacle Detected:</strong> {status.obstacle_detected ? 'Yes' : 'No'}</p>
              <p><strong>Motor Speeds:</strong> {JSON.stringify(status.current_speeds)}</p>
              <p><strong>Last Command:</strong> {status.last_command}</p>
              <p><strong>Uptime:</strong> {status.uptime}s</p>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default ArrowKeys;