import React from 'react';
import { Link } from 'react-router-dom';
import './SelectionPage.css';

const SelectionPage: React.FC = () => {
  return (
    <div className="selection-container">
      <h1>Wybierz wizualizację</h1>
      <div className="buttons-container">
        <Link to="/canvas/2d" className="button-link">
          <button className="selection-button">Wizualizacja 2D</button>
        </Link>
        <Link to="/canvas/3d" className="button-link">
          <button className="selection-button">Wizualizacja 3D</button>
        </Link>
      </div>
    </div>
  );
};

export default SelectionPage;
