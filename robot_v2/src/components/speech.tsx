import React, { useState, useEffect, useRef } from 'react';

interface RobotStatus {
  status: string;
  message: string;
  obstacle_detected: boolean;
  current_speeds: Record<string, number>;
  last_command: string;
  uptime: number;
}

// Discriminated union for messages from FastAPI WS endpoint
type WSMessage =
  | { type: 'status_update'; data: RobotStatus }
  | { type: 'text_command_processed'; data: { message: string; success: boolean } }
  | { type: 'text_command_result'; data: { message: string; success: boolean } }
  | { type: 'direction_executed'; data: { direction: string; success: boolean } }
  | { type: 'robot_stopped'; data: { message: string } }
  | { type: 'error'; data: { message: string } }
  | { type: 'ping' }
  | { type: 'pong' };

const Speech: React.FC = () => {
  const [transcript, setTranscript] = useState('');
  const [finalTranscript, setFinalTranscript] = useState('');
  const [listening, setListening] = useState(false);
  const [recognition, setRecognition] = useState<any>(null);
  const [browserSupported, setBrowserSupported] = useState(false);
  
  const [response, setResponse] = useState<{ message: string; isError: boolean }>({ message: '', isError: false });
  const [status, setStatus] = useState<RobotStatus | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [retryCount, setRetryCount] = useState(0);
  const maxRetries = 5;

  // Use a ref so we always send on the latest socket
  const socketRef = useRef<WebSocket | null>(null);

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
  }, []);

  // WebSocket connection
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
        setResponse({ message: 'Connected to robot', isError: false });
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
          case 'text_command_result':
            setResponse({ message: msg.data.message, isError: !msg.data.success });
            setIsLoading(false);
            break;
          case 'direction_executed':
            setResponse({
              message: `Direction ${msg.data.direction} ${msg.data.success ? 'succeeded' : 'failed'}`,
              isError: !msg.data.success
            });
            setIsLoading(false);
            break;
          case 'robot_stopped':
            setResponse({ message: msg.data.message, isError: false });
            setIsLoading(false);
            break;
          case 'error':
            setResponse({ message: msg.data.message, isError: true });
            setIsLoading(false);
            break;
          // ignore ping/pong
        }
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
      const response = await fetch(`http://${window.location.hostname}:8000/api/text-command`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ text })
      });
      
      const data = await response.json();
      setResponse({ message: data.message, isError: !data.success });
    } catch (err) {
      setResponse({ message: 'Error sending command', isError: true });
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
        const mappedDir = dir === 'up' ? 'forward' : dir === 'down' ? 'backward' : dir;
        socketRef.current.send(JSON.stringify({ 
          type: 'direction_command', 
          data: { direction: mappedDir } 
        }));
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

  const startListening = () => {
    if (recognition && !listening) {
      setTranscript('');
      setFinalTranscript('');
      recognition.start();
    }
  };

  const stopListening = () => {
    if (recognition && listening) {
      recognition.stop();
    }
  };

  const resetTranscript = () => {
    setTranscript('');
    setFinalTranscript('');
    setResponse({ message: '', isError: false });
  };

  if (!browserSupported) {
    return (
      <div style={styles.container}>
        <div style={styles.card}>
          <h2 style={styles.title}>Speech Control</h2>
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
      {/* heartbeat animation */}
      <style>{`@keyframes pulse {0%{opacity:1}50%{opacity:0.5}100%{opacity:1}}`}</style>

      <div style={styles.card}>
        <h2 style={styles.title}>Voice Control</h2>
        <p style={styles.subtitle}>Speak commands to control your robot</p>

        <div style={styles.statusContainer}>
          <span style={styles.statusText}>
            Microphone: {listening ? 'Listening...' : 'Ready'} 
          </span>
          <span
            style={{
              ...styles.indicator,
              background: listening ? '#48bb78' : '#e53e3e'
            }}
          />
        </div>

        <div style={styles.buttonContainer}>
          <button
            onClick={listening ? stopListening : startListening}
            style={{
              ...styles.toggleBtn,
              background: listening ? '#f56565' : '#48bb78'
            }}
            disabled={isLoading}
          >
            {listening ? '🎤 Stop Listening' : '🎤 Start Listening'}
          </button>

          <button
            onClick={resetTranscript}
            style={styles.resetBtn}
          >
            🔄 Reset
          </button>
        </div>

        <div style={styles.transcript}>
          {transcript || finalTranscript || 'Say something to control the robot...'}
          {transcript && <span style={{ color: '#a0aec0' }}> {transcript}</span>}
        </div>

        {isLoading && <div style={styles.loadingMessage}>Processing command…</div>}
        
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
              }}>{status.obstacle_detected ? 'Yes' : 'No'}</span></div>
              <div><strong>Last Command:</strong> {status.last_command || 'None'}</div>
              <div><strong>Uptime:</strong> {status.uptime?.toFixed(1)}s</div>
            </div>
          </div>
        )}

        <div style={styles.examples}>
          <h4 style={styles.examplesTitle}>Example Voice Commands:</h4>
          <ul style={styles.examplesList}>
            <li>"Go forward"</li>
            <li>"Turn left"</li>
            <li>"Move backward 2 seconds"</li>
            <li>"Stop"</li>
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
  statusContainer: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    marginBottom: '1.5rem'
  },
  statusText: { 
    fontSize: '1.1rem',
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
  buttonContainer: {
    display: 'flex',
    gap: '1rem',
    justifyContent: 'center',
    marginBottom: '1.5rem',
    flexWrap: 'wrap' as const
  },
  toggleBtn: {
    padding: '1rem 2rem',
    borderRadius: 12,
    border: 'none',
    cursor: 'pointer',
    color: 'white',
    fontSize: '1rem',
    fontWeight: '600',
    transition: 'all 0.3s ease'
  },
  resetBtn: {
    padding: '1rem 2rem',
    borderRadius: 12,
    border: 'none',
    cursor: 'pointer',
    background: '#ed8936',
    color: 'white',
    fontSize: '1rem',
    fontWeight: '600',
    transition: 'all 0.3s ease'
  },
  transcript: {
    background: '#f7fafc',
    borderRadius: 12,
    padding: '1.5rem',
    minHeight: 80,
    border: '2px solid #e2e8f0',
    margin: '1rem 0',
    fontSize: '1.1rem',
    lineHeight: '1.5',
    color: '#2d3748'
  },
  loadingMessage: {
    color: '#4a5568',
    fontStyle: 'italic',
    marginBottom: '1rem'
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
  },
  errorMessage: {
    textAlign: 'center' as const,
    padding: '2rem'
  }
};

export default Speech;