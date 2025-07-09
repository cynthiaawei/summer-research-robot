import { useState, useEffect } from "react";
import { useNavigate } from "react-router-dom";
import { UserHeader } from './UserContext';

interface HealthStatus {
  status: string;
  robot_available: boolean;
  active_connections: number;
  features: {
    face_recognition: boolean;
    speech_recognition: boolean;
    hand_detection: boolean;
    camera: boolean;
  };
}

const NavButtons = () => {
  const [selectedIndex, setSelectedIndex] = useState(-1);
  const [healthStatus, setHealthStatus] = useState<HealthStatus | null>(null);
  const [loading, setLoading] = useState(true);
  
  const items = [
    {
      name: "Speech Control",
      description: "Voice commands with face recognition",
      icon: "🎤",
      features: ["Face Recognition", "Speech Synthesis", "Voice Commands", "AI Conversations"]
    },
    {
      name: "Text Control",
      description: "Natural language commands with AI chat",
      icon: "💬",
      features: ["Text Commands", "AI Chat", "Command Presets", "Speech Output"]
    },
    {
      name: "Arrow Keys",
      description: "Direct movement control with real-time feedback",
      icon: "🎮",
      features: ["Direct Control", "Camera Feed", "Movement History", "Auto Mode"]
    }
  ];
  
  const navigate = useNavigate();

  // Check robot health status
  useEffect(() => {
    const checkHealthStatus = async () => {
      try {
        const response = await fetch(`http://${window.location.hostname}:8000/api/health`);
        const data = await response.json();
        setHealthStatus(data);
      } catch (error) {
        console.error('Failed to fetch health status:', error);
        setHealthStatus({
          status: 'error',
          robot_available: false,
          active_connections: 0,
          features: {
            face_recognition: false,
            speech_recognition: false,
            hand_detection: false,
            camera: false
          }
        });
      } finally {
        setLoading(false);
      }
    };

    checkHealthStatus();
    // Refresh status every 10 seconds
    const interval = setInterval(checkHealthStatus, 10000);
    return () => clearInterval(interval);
  }, []);

  const containerStyle = {
    minHeight: '100vh',
    width: '100vw',
    background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
    display: 'flex',
    justifyContent: 'center',
    alignItems: 'center',
    fontFamily: '"Inter", sans-serif',
    padding: '2rem',
    paddingTop: '6rem' // Add top padding for UserHeader
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
    maxWidth: '800px'
  };

  const titleStyle = {
    fontSize: '2.5rem',
    fontWeight: '700',
    color: '#2d3748',
    marginBottom: '1rem',
    background: 'linear-gradient(135deg, #667eea, #764ba2)',
    WebkitBackgroundClip: 'text',
    WebkitTextFillColor: 'transparent',
    backgroundClip: 'text'
  };

  const subtitleStyle = {
    color: '#718096',
    marginBottom: '2rem',
    fontSize: '1.2rem',
    lineHeight: '1.6'
  };

  const statusCardStyle = {
    background: 'rgba(247, 250, 252, 0.8)',
    borderRadius: '16px',
    padding: '1.5rem',
    marginBottom: '2rem',
    border: '1px solid rgba(226, 232, 240, 0.5)'
  };

  const statusIndicatorStyle = {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    marginBottom: '1rem',
    fontSize: '1.1rem',
    fontWeight: '600'
  };

  const statusDotStyle = {
    width: '12px',
    height: '12px',
    borderRadius: '50%',
    marginRight: '10px',
    backgroundColor: healthStatus?.status === 'healthy' && healthStatus?.robot_available ? '#48bb78' : '#e53e3e'
  };

  const featuresGridStyle = {
    display: 'grid',
    gridTemplateColumns: 'repeat(auto-fit, minmax(150px, 1fr))',
    gap: '0.5rem',
    fontSize: '0.9rem',
    color: '#4a5568'
  };

  const featureItemStyle = (available: boolean) => ({
    padding: '0.5rem',
    borderRadius: '8px',
    background: available ? 'rgba(72, 187, 120, 0.1)' : 'rgba(229, 62, 62, 0.1)',
    border: `1px solid ${available ? 'rgba(72, 187, 120, 0.3)' : 'rgba(229, 62, 62, 0.3)'}`,
    color: available ? '#2f855a' : '#c53030'
  });

  const listStyle = {
    listStyle: 'none',
    display: 'flex',
    flexDirection: 'column' as const,
    gap: '1rem',
    padding: 0,
    margin: 0
  };

  const getItemStyle = (index: number) => ({
    background: selectedIndex === index 
      ? 'linear-gradient(135deg, #667eea, #764ba2)' 
      : 'rgba(255, 255, 255, 0.9)',
    border: '2px solid transparent',
    borderRadius: '20px',
    padding: '2rem',
    cursor: 'pointer',
    transition: 'all 0.4s cubic-bezier(0.4, 0, 0.2, 1)',
    fontWeight: '600',
    fontSize: '1.1rem',
    color: selectedIndex === index ? 'white' : '#4a5568',
    transform: selectedIndex === index ? 'translateY(-4px) scale(1.02)' : 'none',
    boxShadow: selectedIndex === index 
      ? '0 20px 40px rgba(102, 126, 234, 0.4)' 
      : '0 8px 16px rgba(0, 0, 0, 0.1)',
    backdropFilter: 'blur(10px)',
    position: 'relative' as const,
    overflow: 'hidden'
  });

  const itemIconStyle = {
    fontSize: '2.5rem',
    marginBottom: '1rem',
    display: 'block'
  };

  const itemTitleStyle = {
    fontSize: '1.3rem',
    fontWeight: '700',
    marginBottom: '0.5rem'
  };

  const itemDescriptionStyle = {
    fontSize: '1rem',
    marginBottom: '1rem',
    opacity: 0.9
  };

  const itemFeaturesStyle = {
    display: 'flex',
    flexWrap: 'wrap' as const,
    gap: '0.5rem',
    justifyContent: 'center'
  };

  const featureTagStyle = (isSelected: boolean) => ({
    padding: '0.25rem 0.75rem',
    borderRadius: '12px',
    fontSize: '0.8rem',
    fontWeight: '500',
    background: isSelected ? 'rgba(255, 255, 255, 0.2)' : 'rgba(102, 126, 234, 0.1)',
    color: isSelected ? 'white' : '#667eea',
    border: `1px solid ${isSelected ? 'rgba(255, 255, 255, 0.3)' : 'rgba(102, 126, 234, 0.2)'}`
  });

  const handleItemClick = (index: number, item: any) => {
    setSelectedIndex(index);
    setTimeout(() => {
      if (item.name === "Speech Control") navigate("/speech");
      if (item.name === "Text Control") navigate("/text");
      if (item.name === "Arrow Keys") navigate("/arrow-keys");
    }, 300);
  };

  if (loading) {
    return (
      <div style={containerStyle}>
        <div style={cardStyle}>
          <div style={{ fontSize: '2rem', marginBottom: '1rem' }}>🤖</div>
          <h2>Loading Robot Control Center...</h2>
          <div style={{ 
            width: '50px', 
            height: '50px', 
            border: '3px solid #f3f3f3',
            borderTop: '3px solid #667eea',
            borderRadius: '50%',
            animation: 'spin 1s linear infinite',
            margin: '2rem auto'
          }} />
          <style>{`
            @keyframes spin {
              0% { transform: rotate(0deg); }
              100% { transform: rotate(360deg); }
            }
          `}</style>
        </div>
      </div>
    );
  }

  return (
    <div style={containerStyle}>
      {/* User Header - Shows "Hi {name}" */}
      <div style={{ position: 'fixed', top: '1rem', left: '1rem', right: '1rem', zIndex: 1000 }}>
        <UserHeader />
      </div>

      <div style={cardStyle}>
        <h1 style={titleStyle}>Enhanced Robot Control Center</h1>
        <p style={subtitleStyle}>
          Advanced robot interface with face recognition, speech synthesis, and intelligent control
        </p>

        {/* Robot Status */}
        <div style={statusCardStyle}>
          <div style={statusIndicatorStyle}>
            <div style={statusDotStyle}></div>
            <span>
              Robot Status: {healthStatus?.status === 'healthy' && healthStatus?.robot_available ? 'Online' : 'Offline'}
            </span>
          </div>
          
          <div style={{ marginBottom: '1rem', fontSize: '0.9rem', color: '#4a5568' }}>
            Active Connections: {healthStatus?.active_connections || 0} | 
            Robot Available: {healthStatus?.robot_available ? '✅' : '❌'}
          </div>

          <div style={featuresGridStyle}>
            <div style={featureItemStyle(healthStatus?.features.face_recognition || false)}>
              👁️ Face Recognition
            </div>
            <div style={featureItemStyle(healthStatus?.features.speech_recognition || false)}>
              🎤 Speech Recognition
            </div>
            <div style={featureItemStyle(healthStatus?.features.hand_detection || false)}>
              ✋ Hand Detection
            </div>
            <div style={featureItemStyle(healthStatus?.features.camera || false)}>
              📷 Camera Feed
            </div>
          </div>
        </div>

        {/* Control Options */}
        <ul style={listStyle}>
          {items.map((item, index) => (
            <li
              key={item.name}
              style={getItemStyle(index)}
              onClick={() => handleItemClick(index, item)}
              onMouseEnter={(e) => {
                if (selectedIndex !== index) {
                  const target = e.target as HTMLLIElement;
                  target.style.transform = 'translateY(-6px) scale(1.01)';
                  target.style.boxShadow = '0 16px 32px rgba(102, 126, 234, 0.3)';
                }
              }}
              onMouseLeave={(e) => {
                if (selectedIndex !== index) {
                  const target = e.target as HTMLLIElement;
                  target.style.transform = 'none';
                  target.style.boxShadow = '0 8px 16px rgba(0, 0, 0, 0.1)';
                }
              }}
            >
              <div style={itemIconStyle}>{item.icon}</div>
              <div style={itemTitleStyle}>{item.name}</div>
              <div style={itemDescriptionStyle}>{item.description}</div>
              <div style={itemFeaturesStyle}>
                {item.features.map((feature, idx) => (
                  <span key={idx} style={featureTagStyle(selectedIndex === index)}>
                    {feature}
                  </span>
                ))}
              </div>
            </li>
          ))}
        </ul>

        {/* Instructions */}
        <div style={{
          marginTop: '2rem',
          padding: '1.5rem',
          background: 'rgba(240, 244, 248, 0.8)',
          borderRadius: '16px',
          textAlign: 'left' as const,
          fontSize: '0.9rem',
          color: '#4a5568'
        }}>
          <h4 style={{ marginTop: 0, color: '#2d3748', fontSize: '1.1rem' }}>🚀 Getting Started:</h4>
          <ul style={{ margin: 0, paddingLeft: '1.5rem', lineHeight: '1.6' }}>
            <li><strong>Speech Control:</strong> Best for hands-free operation with voice commands and face recognition</li>
            <li><strong>Text Control:</strong> Ideal for precise commands and AI conversations</li>
            <li><strong>Arrow Keys:</strong> Perfect for direct manual control with visual feedback</li>
          </ul>
          
          <div style={{ marginTop: '1rem', padding: '1rem', background: 'rgba(102, 126, 234, 0.1)', borderRadius: '8px' }}>
            <strong>💡 Pro Tip:</strong> Start with "Speech Control" if you have a camera and microphone, 
            or use "Arrow Keys" for immediate manual control. All modes support obstacle detection and emergency stop.
          </div>
        </div>

        {/* Version Info */}
        <div style={{
          marginTop: '2rem',
          fontSize: '0.8rem',
          color: '#a0aec0',
          borderTop: '1px solid rgba(226, 232, 240, 0.5)',
          paddingTop: '1rem'
        }}>
          Enhanced Robot Control System v2.0 | Features: Face Recognition, Speech Synthesis, Hand Gesture Detection
        </div>
      </div>
    </div>
  );
};

export default NavButtons;