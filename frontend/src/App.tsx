import { Routes, Route } from 'react-router-dom';
import SelectionPage from './pages/SelectionPage';
import CanvasPage from './pages/CanvasPage';
import './App.css';

function App() {
  return (
    <Routes>
      <Route path="/" element={<SelectionPage />} />
      <Route path="/canvas/:type" element={<CanvasPage />} />
    </Routes>
  );
}

export default App;
