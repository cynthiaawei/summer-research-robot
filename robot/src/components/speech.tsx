import React, { useState, useEffect, useRef } from 'react';
import SpeechRecognition, { useSpeechRecognition } from 'react-speech-recognition';
import axios, { isAxiosError } from 'axios';

interface RobotStatus {
  status: string;
  message: string;
  obstacle_detected: boolean;
  current_speeds: Record<string, number>;
  last_command: string;
  uptime: number;
}

// Discriminated union for messages from your FastAPI WS endpoint
type WSMessage =
  | { type: 'status_update'; data: RobotStatus }
  | { type: 'text_command_processed'; data: { message: string; success: boolean } }
  | { type: 'direction_executed'; data: { direction: string; success: boolean } }
  | { type: 'robot_stopped'; data: { message: string } }
  | { type: 'error'; data: { message: string } }
  | { type: 'ping' }
  | { type: 'pong' };

const Speech: React.FC = () => {
  const {
    transcript,
    finalTranscript,
    listening,
    resetTranscript,
    browserSupportsSpeechRecognition
  } = useSpeechRecognition();

  const [response, setResponse] = useState<{ message: string; isError: boolean }>({ message: '', isError: false });
  const [status, setStatus] = useState<RobotStatus | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [retryCount, setRetryCount] = useState(0);
  const maxRetries = 5;

  // Use a ref so we always send on the latest socket
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
        console.log('WebSocket connected');
        setResponse({ message: '', isError: false });
        setRetryCount(0);

        // Heartbeat
        pingId = window.setInterval(() => {
          if (ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ type: 'ping' }));
          }
        }, 30000);
      };

      ws.onmessage = (event) => {
        let msg: WSMessage;
        try {
          msg = JSON.parse(event.data);
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
          // ignore ping/pong
        }

        setIsLoading(false);
      };

      ws.onerror = () => {
        console.error('WebSocket error');
        setResponse({ message: 'WebSocket error', isError: true });
        setIsLoading(false);
      };

      ws.onclose = () => {
        console.warn('WebSocket closed, retrying...');
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

  // Fallback HTTP for natural-language commands
  const sendTextCommandViaHTTP = async (text: string) => {
    try {
      const res = await axios.post<{ success: boolean; message: string }>(
        `http://${window.location.hostname}:8000/api/text-command`,
        { text }
      );
      setResponse({ message: res.data.message, isError: !res.data.success });
    } catch (err) {
      let msg = 'Error sending command';
      if (isAxiosError(err)) {
        // our FastAPI errors come back as { detail: string }
        const data = err.response?.data as { detail?: string };
        msg = data.detail ?? err.message;
      }
      setResponse({ message: msg, isError: true });
    } finally {
      setIsLoading(false);
    }
  };

  const handleSpeechCommand = (text: string) => {
    setIsLoading(true);
    const trimmed = text.trim();
    if (!trimmed) {
      setResponse({ message: 'Empty transcript', isError: true });
      setIsLoading(false);
      return;
    }

    const lower = trimmed.toLowerCase();
    const dirs = ['forward', 'backward', 'left', 'right', 'up', 'down'];
    const isMove = dirs.some(d => lower.includes(`go ${d}`) || lower.includes(`move ${d}`));
    const isStop = lower.includes('stop');

    if (isMove) {
      const dir = dirs.find(d => lower.includes(d));
      if (dir && socketRef.current?.readyState === WebSocket.OPEN) {
        socketRef.current.send(JSON.stringify({ type: 'direction_command', data: { direction: dir } }));
        setIsLoading(false);
        return;
      }
    }

    if (isStop && socketRef.current?.readyState === WebSocket.OPEN) {
      socketRef.current.send(JSON.stringify({ type: 'stop_command' }));
      setIsLoading(false);
      return;
    }

    // otherwise, HTTP fallback
    sendTextCommandViaHTTP(lower);
  };

  const handleSpeechEnd = () => {
    if (finalTranscript) {
      handleSpeechCommand(finalTranscript);
      resetTranscript();
    } else {
      setIsLoading(false);
    }
  };

  if (!browserSupportsSpeechRecognition) {
    return (
      <div style={styles.container}>
        <div style={styles.card}>
          <span style={{ color: '#e53e3e' }}>Browser doesn’t support speech recognition</span>
        </div>
      </div>
    );
  }

  return (
    <div style={styles.container}>
      {/* heartbeat animation */}
      <style>{`@keyframes pulse {0%{opacity:1}50%{opacity:0.5}100%{opacity:1}}`}</style>

      <div style={styles.card}>
        <p style={styles.status}>
          Mic: {listening ? 'on' : 'off'} 
          <span
            style={{
              ...styles.indicator,
              background: listening ? '#48bb78' : '#e53e3e'
            }}
          />
        </p>

        <button
          onClick={() => {
            if (!listening) {
              SpeechRecognition.startListening({ continuous: true, interimResults: true });
            } else {
              SpeechRecognition.stopListening();
              handleSpeechEnd();
            }
          }}
          style={styles.toggleBtn}
        >
          {listening ? 'Stop Listening' : 'Start Listening'}
        </button>

        <button
          onClick={() => {
            resetTranscript();
            setResponse({ message: '', isError: false });
          }}
          style={styles.resetBtn}
        >
          Reset
        </button>

        <div style={styles.transcript}>{transcript || 'Say something to control the robot...'}</div>

        {isLoading && <div style={styles.response}>Processing command…</div>}
        {!isLoading && response.message && (
          <div
            style={{
              ...styles.response,
              background: response.isError ? '#fee2e2' : '#d1fae5',
              borderColor: response.isError ? '#f87171' : '#34d399'
            }}
          >
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
            <p><strong>Last Command:</strong> {status.last_command}</p>
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
    background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
    display: 'flex' as const,
    justifyContent: 'center',
    alignItems: 'center',
    padding: '2rem'
  },
  card: {
    background: 'rgba(255,255,255,0.95)',
    borderRadius: '24px',
    padding: '2rem',
    boxShadow: '0 20px 40px rgba(0,0,0,0.1)',
    textAlign: 'center' as const,
    maxWidth: '600px',
    width: '100%'
  },
  status: { fontSize: '1.2rem', marginBottom: '1rem' },
  indicator: {
    display: 'inline-block',
    width: 12,
    height: 12,
    borderRadius: '50%',
    marginLeft: 8,
    animation: 'pulse 2s infinite'
  },
  toggleBtn: {
    padding: '1rem 2rem',
    borderRadius: 12,
    border: 'none',
    margin: '0.5rem',
    cursor: 'pointer',
    background: '#48bb78',
    color: 'white'
  },
  resetBtn: {
    padding: '1rem 2rem',
    borderRadius: 12,
    border: 'none',
    margin: '0.5rem',
    cursor: 'pointer',
    background: '#ed8936',
    color: 'white'
  },
  transcript: {
    background: '#f7fafc',
    borderRadius: 12,
    padding: '1rem',
    minHeight: 80,
    border: '2px solid #e2e8f0',
    margin: '1rem 0'
  },
  response: {
    borderRadius: 12,
    padding: '1rem',
    border: '2px solid',
    marginBottom: '1rem'
  },
  statusCard: {
    background: '#ffffff',
    borderRadius: 12,
    padding: '1rem',
    border: '2px solid #e2e8f0',
    textAlign: 'left' as const,
    marginTop: '1rem'
  }
};

export default Speech;
