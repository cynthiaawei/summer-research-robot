import React from 'react';
import { Routes, Route } from 'react-router-dom';
import { UserProvider } from './components/UserContext';
import FaceRecognitionGate from './components/facerecognition';
import NavButtons from './components/navbuttons';
import TextButton from './components/textbutton';
import Speech from './components/speech';
import ArrowKeys from './components/arrow_keys';

const App: React.FC = () => {
  return (
    <UserProvider>
      <Routes>
        <Route path="/" element={<FaceRecognitionGate />} />
        <Route path="/menu" element={<NavButtons />} />
        <Route path="/text" element={<TextButton />} />
        <Route path="/speech" element={<Speech />} />
        <Route path="/arrow-keys" element={<ArrowKeys />} />
      </Routes>
    </UserProvider>
  );
}; 

export default App;