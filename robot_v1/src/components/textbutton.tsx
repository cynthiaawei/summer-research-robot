import { useState } from "react";

const TextButton = () => {
  const [text, setText] = useState("");

  const press_button = async () => {
    if (text) {
      await fetch(`http://${window.location.hostname}:8000/api/text-command`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ text }),
      });
      setText(""); // Clear input after sending
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
    border: '1px solid rgba(255, 255, 255, 0.2)',
    width: '100%',
    maxWidth: '600px'
  };

  const inputGroupStyle = {
    position: 'relative' as const,
    marginBottom: '1rem'
  };

  const labelStyle = {
    display: 'block',
    fontSize: '1.2rem',
    fontWeight: '600',
    color: '#4a5568',
    marginBottom: '0.5rem',
    background: 'linear-gradient(135deg, #667eea, #764ba2)',
    WebkitBackgroundClip: 'text',
    WebkitTextFillColor: 'transparent',
    backgroundClip: 'text'
  };

  const inputStyle = {
    width: '100%',
    padding: '1.5rem',
    border: '2px solid #e2e8f0',
    borderRadius: '16px',
    fontSize: '1.1rem',
    color: 'black',
    background: '#f7fafc',
    transition: 'all 0.3s ease',
    outline: 'none',
    boxSizing: 'border-box' as const
  };

  const hintStyle = {
    fontSize: '0.9rem',
    color: '#718096',
    textAlign: 'center' as const,
    marginTop: '1rem'
  };

  const handleFocus = (e: React.FocusEvent<HTMLInputElement>) => {
    e.target.style.borderColor = '#667eea';
    e.target.style.background = 'white';
    e.target.style.transform = 'translateY(-2px)';
    e.target.style.boxShadow = '0 8px 16px rgba(102, 126, 234, 0.2)';
  };

  const handleBlur = (e: React.FocusEvent<HTMLInputElement>) => {
    e.target.style.borderColor = '#e2e8f0';
    e.target.style.background = '#f7fafc';
    e.target.style.transform = 'translateY(0)';
    e.target.style.boxShadow = 'none';
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === "Enter") {
      press_button();
      e.currentTarget.style.transform = 'translateY(-4px)';
      setTimeout(() => {
        e.currentTarget.style.transform = 'translateY(-2px)';
      }, 100);
    }
  };

  return (
    <div style={containerStyle}>
      <div style={cardStyle}>
        <div style={inputGroupStyle}>
          <label style={labelStyle}>Command</label>
          <input
            type="text"
            style={inputStyle}
            placeholder="Enter your command here..."
            value={text} // Controlled component
            onChange={(e) => setText(e.target.value)}
            onKeyDown={handleKeyDown}
            onFocus={handleFocus}
            onBlur={handleBlur}
          />
        </div>
        <p style={hintStyle}>Press Enter to submit your command</p>
      </div>
    </div>
  );
};

export default TextButton;