import React, { useState, useEffect, useRef } from 'react';
import axios, { isAxiosError } from 'axios';

interface RobotStatus {
  status: string;
  message: string;
  obstacle_detected: boolean;
  current_speeds: Record<string, number>;
  last_command: string;
  uptime: number;
}

// Discriminated union for WS messages
type WSMessage =
  | { type: 'status_update'; data: RobotStatus }
  | { type: 'text_command_processed'; data: { message: string; success: boolean } }
  | { type: 'direction_executed'; data: { direction: string; success: boolean } }
  | { type: 'robot_stopped'; data: { message: string } }
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
        setResponse({ message: '', isError: false });
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
        data: { message: cmd }    // your backend will look for `data.text` or `data.message`
      }));
      setIsLoading(false);
      return;
    }

    // Otherwise HTTP fallback
    sendTextCommandViaHTTP(cmd);
  };

  const handleKeyDown = (e: React.KeyboardEvent, action: 'submit') => {
    if (e.key === 'Enter' && action === 'submit') {
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
        <label htmlFor="cmd" style={styles.label}>Command</label>
        <input
          id="cmd"
          type="text"
          value={text}
          onChange={e => setText(e.target.value)}
          onKeyDown={e => handleKeyDown(e, 'submit')}
          placeholder="e.g. go forward 2 seconds"
          style={styles.input}
        />

        <button
          onClick={handleSubmit}
          onKeyDown={e => handleKeyDown(e, 'submit')}
          disabled={isLoading}
          style={styles.button}
        >
          {isLoading ? 'Sending…' : 'Submit'}
        </button>

        {response.message && (
          <div style={{
            ...styles.response,
            background: response.isError ? '#fee2e2' : '#d1fae5',
            borderColor: response.isError ? '#f87171' : '#34d399'
          }}>
            {response.message}
          </div>
        )}

        {status && (
          <div style={styles.statusCard}>
            <h3>Robot Status</h3>
            <p><strong>Status:</strong> {status.status}</p>
            <p><strong>Message:</strong> {status.message}</p>
            <p><strong>Obstacle:</strong> {status.obstacle_detected ? 'Yes' : 'No'}</p>
            <p><strong>Speeds:</strong> {JSON.stringify(status.current_speeds)}</p>
            <p><strong>Last Cmd:</strong> {status.last_command}</p>
            <p><strong>Uptime:</strong> {status.uptime.toFixed(1)}s</p>
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
  label: {
    fontSize: '1.2rem',
    fontWeight: 600,
    marginBottom: '0.5rem',
    display: 'block'
  },
  input: {
    width: '100%',
    padding: '1rem',
    fontSize: '1rem',
    borderRadius: 12,
    border: '2px solid #e2e8f0',
    marginBottom: '1rem',
    boxSizing: 'border-box' as const
  },
  button: {
    padding: '0.75rem 1.5rem',
    borderRadius: 12,
    border: 'none',
    background: '#48bb78',
    color: 'white',
    cursor: 'pointer'
  },
  response: {
    marginTop: '1rem',
    padding: '1rem',
    border: '2px solid',
    borderRadius: 12,
    fontSize: '0.9rem'
  },
  statusCard: {
    marginTop: '1.5rem',
    padding: '1rem',
    border: '2px solid #e2e8f0',
    borderRadius: 12,
    textAlign: 'left' as const
  }
};

export default TextButton;
