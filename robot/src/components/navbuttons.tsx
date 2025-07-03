import { useState } from "react";
import { useNavigate } from "react-router-dom";

const NavButtons = () => {
  const [selectedIndex, setSelectedIndex] = useState(-1);
  const items = ["Speech", "Text", "Arrow_keys"];
  const navigate = useNavigate();

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
    maxWidth: '500px'
  };

  const titleStyle = {
    fontSize: '2.5rem',
    fontWeight: '700',
    color: '#2d3748',
    marginBottom: '2rem',
    background: 'linear-gradient(135deg, #667eea, #764ba2)',
    WebkitBackgroundClip: 'text',
    WebkitTextFillColor: 'transparent',
    backgroundClip: 'text'
  };

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
      : 'linear-gradient(135deg, #f7fafc, #edf2f7)',
    border: '2px solid transparent',
    borderRadius: '16px',
    padding: '1.5rem 2rem',
    cursor: 'pointer',
    transition: 'all 0.3s cubic-bezier(0.4, 0, 0.2, 1)',
    fontWeight: '600',
    fontSize: '1.1rem',
    color: selectedIndex === index ? 'white' : '#4a5568',
    transform: selectedIndex === index ? 'translateY(-2px)' : 'none',
    boxShadow: selectedIndex === index 
      ? '0 8px 16px rgba(102, 126, 234, 0.4)' 
      : '0 2px 4px rgba(0, 0, 0, 0.1)'
  });

  const handleItemClick = (index: number, item: string) => {
    setSelectedIndex(index);
    setTimeout(() => {
      if (item === "Speech") navigate("/Speech");
      if (item === "Text") navigate("/Text");
      if (item === "Arrow_keys") navigate("/arrow_keys");
    }, 200);
  };

  return (
    <div style={containerStyle}>
      <div style={cardStyle}>
        <h1 style={titleStyle}>Choose an Input Method</h1>
        <ul style={listStyle}>
          {items.map((item, index) => (
            <li
              key={item}
              style={getItemStyle(index)}
              onClick={() => handleItemClick(index, item)}
              onMouseEnter={(e) => {
                if (selectedIndex !== index) {
                  const target = e.target as HTMLLIElement;
                  target.style.transform = 'translateY(-4px)';
                  target.style.boxShadow = '0 12px 24px rgba(102, 126, 234, 0.3)';
                }
              }}
              onMouseLeave={(e) => {
                if (selectedIndex !== index) {
                  const target = e.target as HTMLLIElement;
                  target.style.transform = 'none';
                  target.style.boxShadow = '0 2px 4px rgba(0, 0, 0, 0.1)';
                }
              }}
            >
              {item}
            </li>
          ))}
        </ul>
      </div>
    </div>
  );
};

export default NavButtons;