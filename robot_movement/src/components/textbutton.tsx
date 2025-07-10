import React, { useState, useEffect, useRef } from 'react';
import axios, { isAxiosError } from 'axios';

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
}

// Discriminated union for WS messages
type WSMessage =
  | { type: 'status_update'; data: RobotStatus }
  | { type: 'text_command_processed'; data: { message: string; success: boolean } }
  | { type: 'text_command_result'; data: { message: string; success: boolean } }
  | { type: 'direction_executed'; data: { direction: string; success: boolean } }
  | { type: 'robot_stopped'; data: { message: string } }
  | { type: 'obstacle_reset'; data: { message: string; success: boolean } }
  | { type: 'error'; data: { message: string } }
  | { type: 'ping' }
  | { type: 'pong' };

const TextButton: React.FC = () => {
  const [text, setText] = useState('');
  const [response, setResponse] = useState<{ message: string; isError: boolean }>({ message: '', isError: false });
  const [status, setStatus] = useState<RobotStatus | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [retryCount, setRetryCount] = useState(0);
  const maxRetries = 5;

  // socketRef always points to the latest WebSocket instance
  const socketRef = useRef<WebSocket | null>(null);

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
        setResponse({ message: 'Connected to robot', isError: false });
        setRetryCount(0);

        // keep-alive ping every 30s
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
          // ping/pong can be ignored
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

  // Fallback HTTP call
  const sendTextCommandViaHTTP = async (cmd: string) => {
    try {
      const res = await axios.post<{ success: boolean; message: string }>(
        `http://${window.location.hostname}:8000/api/text-command`,
        { text: cmd }
      );
      setResponse({ message: res.data.message, isError: !res.data.success });
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
    setText('');

    // If WS open, send over socket
    if (socketRef.current?.readyState === WebSocket.OPEN) {
      socketRef.current.send(JSON.stringify({
        type: 'text_command',
        data: { text: cmd }
      }));
      // Don't set loading to false here, wait for response
      return;
    }

    // Otherwise HTTP fallback
    sendTextCommandViaHTTP(cmd);
  };

  const resetObstacle = () => {
    if (socketRef.current?.readyState === WebSocket.OPEN) {
      socketRef.current.send(JSON.stringify({ type: 'reset_obstacle' }));
      setResponse({ message: 'Obstacle reset requested...', isError: false });
    } else {
      setResponse({ message: 'WebSocket not connected', isError: true });
    }
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

      <div style={styles.card}>
        <h2 style={styles.title}>Text Command Control</h2>
        <p style={styles.subtitle}>Enter natural language commands to control your robot</p>
        
        <label htmlFor="cmd" style={styles.label}>Command</label>
        <input
          id="cmd"
          type="text"
          value={text}
          onChange={e => setText(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder="e.g. go forward 2 seconds, turn left, stop"
          style={styles.input}
          disabled={isLoading}
        />

        <div style={{ display: 'flex', gap: '1rem', justifyContent: 'center', marginBottom: '1rem' }}>
          <button
            onClick={handleSubmit}
            disabled={isLoading || !text.trim()}
            style={{
              padding: '0.75rem 2rem',
              borderRadius: 12,
              border: 'none',
              color: 'white',
              cursor: isLoading || !text.trim() ? 'not-allowed' : 'pointer',
              fontSize: '1.1rem',
              fontWeight: '600',
              transition: 'all 0.3s ease',
              background: isLoading || !text.trim() ? '#a0aec0' : '#48bb78'
            }}
          >
            {isLoading ? 'Sending…' : 'Send Command'}
          </button>
          
          {status?.obstacle_detected && (
            <button
              onClick={resetObstacle}
              style={{
                padding: '0.75rem 1.5rem',
                borderRadius: 12,
                border: 'none',
                color: 'white',
                cursor: 'pointer',
                fontSize: '1.1rem',
                fontWeight: '600',
                transition: 'all 0.3s ease',
                background: 'linear-gradient(135deg, #ed8936, #dd6b20)'
              }}
            >
              Reset Obstacle
            </button>
          )}
        </div>

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

        {status && (
          <div style={styles.statusCard}>
            <h3 style={styles.statusTitle}>Robot Status</h3>
            <div style={styles.statusGrid}>
              <div><strong>Status:</strong> <span style={{
                color: status.obstacle_detected ? '#e53e3e' : 
                      status.status === 'moving' ? '#38a169' : '#718096'
              }}>{status.status}</span></div>
              <div><strong>Message:</strong> {status.message}</div>
              <div><strong>Obstacle:</strong> <span style={{
                color: status.obstacle_detected ? '#e53e3e' : '#38a169'
              }}>{status.obstacle_detected ? 
                `Yes - Sensor ${status.obstacle_sensor === 'front' ? '1' : 
                                status.obstacle_sensor === 'left' ? '2' : 
                                status.obstacle_sensor === 'right' ? '3' : '?'}` : 'No'}</span></div>
              <div><strong>Last Cmd:</strong> {status.last_command || 'None'}</div>
              <div><strong>Uptime:</strong> {status.uptime?.toFixed(1)}s</div>
            </div>
          </div>
        )}

        <div style={styles.examples}>
          <h4 style={styles.examplesTitle}>Example Commands:</h4>
          <ul style={styles.examplesList}>
            <li>"go forward 3 seconds"</li>
            <li>"turn left"</li>
            <li>"move backward 1 second"</li>
            <li>"stop"</li>
            <li>"move left 2 seconds then turn right"</li>
            <li>Sensors: 1=Front, 2=Left, 3=Right</li>
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
    maxWidth: 600,
    width: '100%',
    textAlign: 'center' as const
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
  label: {
    fontSize: '1.2rem',
    fontWeight: 600,
    marginBottom: '0.5rem',
    display: 'block',
    textAlign: 'left' as const
  },
  input: {
    width: '100%',
    padding: '1rem',
    fontSize: '1rem',
    borderRadius: 12,
    border: '2px solid #e2e8f0',
    marginBottom: '1rem',
    boxSizing: 'border-box' as const,
    transition: 'all 0.3s ease'
  },
  statusCard: {
    marginTop: '1.5rem',
    padding: '1rem',
    border: '2px solid #e2e8f0',
    borderRadius: 12,
    textAlign: 'left' as const
  },
  statusTitle: {
    marginTop: 0,
    marginBottom: '0.5rem',
    fontSize: '1.1rem',
    color: '#2d3748'
  },
  statusGrid: {
    display: 'grid',
    gridTemplateColumns: '1fr 1fr',
    gap: '0.5rem',
    fontSize: '0.9rem'
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