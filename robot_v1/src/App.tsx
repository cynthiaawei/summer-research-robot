import NavButtons from "./components/navbuttons";
import TextButtons from "./components/textbutton";
import Speech from "./components/speech";
import Arrow_keys from "./components/arrow_keys";
import {Routes, Route} from "react-router-dom"
function App() {
  return (
    <Routes>
      <Route path="/" element={<NavButtons />}/>
      <Route path="/Text" element={<TextButtons />}/>
      <Route path="/Speech" element={<Speech />}/>
      <Route path="/Arrow_keys" element={<Arrow_keys/>}/>
    </Routes>
  );
}

export default App
