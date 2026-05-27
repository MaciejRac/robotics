import React from 'react';
import { useParams } from 'react-router-dom';

const CanvasPage: React.FC = () => {
  const { type } = useParams<{ type: string }>();

  return (
    <div>
      <h1>Wizualizacja {type?.toUpperCase()}</h1>
      <canvas id="robot-canvas" width="800" height="600" style={{ border: '1px solid black' }}></canvas>
      {/* Tutaj będzie logika rysowania na canvasie */}
    </div>
  );
};

export default CanvasPage;
