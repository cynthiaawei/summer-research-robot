import SpeechRecognition, { useSpeechRecognition } from 'react-speech-recognition';

const Speech = () => {
  const { transcript, listening, resetTranscript, browserSupportsSpeechRecognition } = useSpeechRecognition();

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
    textAlign: 'center' as const,
    width: '100%',
    maxWidth: '600px'
  };

  const statusStyle = {
    fontSize: '1.5rem',
    fontWeight: '600',
    marginBottom: '2rem',
    color: '#4a5568',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    gap: '0.5rem'
  };

  const indicatorStyle = {
    width: '12px',
    height: '12px',
    borderRadius: '50%',
    background: listening ? '#48bb78' : '#e53e3e',
    animation: 'pulse 2s infinite'
  };

  const controlsStyle = {
    display: 'flex',
    gap: '1rem',
    justifyContent: 'center',
    marginBottom: '2rem',
    flexWrap: 'wrap' as const
  };

  const buttonBaseStyle = {
    padding: '1rem 2rem',
    border: 'none',
    borderRadius: '12px',
    fontSize: '1rem',
    fontWeight: '600',
    cursor: 'pointer',
    transition: 'all 0.3s cubic-bezier(0.4, 0, 0.2, 1)',
    minWidth: '120px'
  };

  const startButtonStyle = {
    ...buttonBaseStyle,
    background: 'linear-gradient(135deg, #48bb78, #38a169)',
    color: 'white',
    boxShadow: '0 4px 12px rgba(72, 187, 120, 0.3)'
  };

  const stopButtonStyle = {
    ...buttonBaseStyle,
    background: 'linear-gradient(135deg, #e53e3e, #c53030)',
    color: 'white',
    boxShadow: '0 4px 12px rgba(229, 62, 62, 0.3)'
  };

  const resetButtonStyle = {
    ...buttonBaseStyle,
    background: 'linear-gradient(135deg, #ed8936, #dd6b20)',
    color: 'white',
    boxShadow: '0 4px 12px rgba(237, 137, 54, 0.3)'
  };

  const transcriptStyle = {
    background: '#f7fafc',
    borderRadius: '12px',
    padding: '1.5rem',
    minHeight: '100px',
    fontSize: '1.1rem',
    color: 'black',
    border: '2px solid #e2e8f0',
    lineHeight: '1.6'
  };

  const handleButtonHover = (e: React.MouseEvent<HTMLButtonElement>) => {
    e.currentTarget.style.transform = 'translateY(-2px)';
    const currentStyle = getComputedStyle(e.currentTarget);
    const currentShadow = currentStyle.boxShadow;
    if (currentShadow.includes('72, 187, 120')) {
      e.currentTarget.style.boxShadow = '0 8px 16px rgba(72, 187, 120, 0.4)';
    } else if (currentShadow.includes('229, 62, 62')) {
      e.currentTarget.style.boxShadow = '0 8px 16px rgba(229, 62, 62, 0.4)';
    } else {
      e.currentTarget.style.boxShadow = '0 8px 16px rgba(237, 137, 54, 0.4)';
    }
  };

  const handleButtonLeave = (e: React.MouseEvent<HTMLButtonElement>) => {
    e.currentTarget.style.transform = 'translateY(0)';
    const currentStyle = getComputedStyle(e.currentTarget);
    const currentShadow = currentStyle.boxShadow;
    if (currentShadow.includes('72, 187, 120')) {
      e.currentTarget.style.boxShadow = '0 4px 12px rgba(72, 187, 120, 0.3)';
    } else if (currentShadow.includes('229, 62, 62')) {
      e.currentTarget.style.boxShadow = '0 4px 12px rgba(229, 62, 62, 0.3)';
    } else {
      e.currentTarget.style.boxShadow = '0 4px 12px rgba(237, 137, 54, 0.3)';
    }
  };

  const handleSpeechEnd = async () => {
    if (transcript) {
      await fetch(`http://${window.location.hostname}:8000/api/text-command`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ text: transcript }),
      });
      resetTranscript();
    }
  };

  if (!browserSupportsSpeechRecognition) {
    return (
      <div style={containerStyle}>
        <div style={cardStyle}>
          <span style={{ fontSize: '1.2rem', color: '#e53e3e' }}>
            Browser doesn't support speech recognition
          </span>
        </div>
      </div>
    );
  }
  console.log(transcript, listening);

  return (
    <div style={containerStyle}>
      <style>
        {`
          @keyframes pulse {
            0% { opacity: 1; }
            50% { opacity: 0.5; }
            100% { opacity: 1; }
          }
        `}
      </style>
      <div style={cardStyle}>
        <p style={statusStyle}>
          Microphone: {listening ? "on" : "off"}
          <span style={indicatorStyle}></span>
        </p>
        <div style={controlsStyle}>
          <button 
            style={startButtonStyle}
            onClick={() => SpeechRecognition.startListening()}
            onMouseEnter={handleButtonHover}
            onMouseLeave={handleButtonLeave}
          >
            Start
          </button>
          <button 
            style={stopButtonStyle}
            onClick={() => { SpeechRecognition.stopListening(); handleSpeechEnd(); }}
            onMouseEnter={handleButtonHover}
            onMouseLeave={handleButtonLeave}
          >
            Stop
          </button>
          <button 
            style={resetButtonStyle}
            onClick={resetTranscript}
            onMouseEnter={handleButtonHover}
            onMouseLeave={handleButtonLeave}
          >
            Reset
          </button>
        </div>
        <div style={transcriptStyle}>
          {transcript}
        </div>
      </div>
    </div>
  );
};

export default Speech;