import React from 'react';
import {Routes, Route } from 'react-router-dom';
import NavButtons from './components/navbuttons';
import TextButton from './components/textbutton';
import Speech from './components/speech';
import ArrowKeys from './components/arrow_keys';

const App: React.FC = () => {
  return (
      <Routes>
        <Route path="/" element={<NavButtons />} />
        <Route path="/text" element={<TextButton />} />
        <Route path="/speech" element={<Speech />} />
        <Route path="/arrow-keys" element={<ArrowKeys />} />
      </Routes>
  );
}; 

export default App;