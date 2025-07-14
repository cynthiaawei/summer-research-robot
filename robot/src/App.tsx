import React from 'react';
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import FaceRecognitionGate from './components/facerecognition';
import NavButtons from './components/navbuttons';
import TextButton from './components/textbutton';
import Speech from './components/speech';
import ArrowKeys from './components/arrow_keys';
import RegistrationPage from './components/UserRegistration';

const App: React.FC = () => {
  console.log('🚀 App component rendering');
  
  return (
    <Router>
      <div style={{ minHeight: '100vh', width: '100vw' }}>
        <Routes>
          {/* Face Recognition Gate - Entry Point */}
          <Route path="/" element={<FaceRecognitionGate />} />
          
          {/* User Registration - When face recognition fails after 3 attempts */}
          <Route path="/register" element={<RegistrationPage />} />
          
          {/* Main Menu - After successful face recognition or registration */}
          <Route path="/menu" element={<NavButtons />} />
          
          {/* Control Interfaces */}
          <Route path="/text" element={<TextButton />} />
          <Route path="/speech" element={<Speech />} />
          <Route path="/arrow-keys" element={<ArrowKeys />} />
          
          {/* Legacy route compatibility */}
          <Route path="/user" element={<ArrowKeys />} />
        </Routes>
      </div>
    </Router>
  );
}; 

export default App;