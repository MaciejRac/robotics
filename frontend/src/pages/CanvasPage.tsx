import React, { useEffect, useRef, useState } from 'react';
import { useParams, useNavigate } from 'react-router-dom';
import * as THREE from 'three';
import './CanvasPage.css';

const CanvasPage: React.FC = () => {
  const { type } = useParams<{ type: string }>();
  const navigate = useNavigate();
  const containerRef = useRef<HTMLDivElement>(null);
  const [latency, setLatency] = useState<number>(0);
  const [maxLatency, setMaxLatency] = useState<number>(0);
  const latencySamplesRef = useRef<number[]>([]);

  useEffect(() => {
    // 1. Zapisujemy aktualną referencję do zmiennej!
    const currentContainer = containerRef.current;
    
    if (type !== '3d' || !currentContainer) return;

    // --- Inicjalizacja sceny Three.js ---
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf0f0f0);

    const camera = new THREE.PerspectiveCamera(75, 800 / 600, 0.1, 1000);
    camera.position.set(5, 5, 5);
    camera.lookAt(0, 0, 0);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(800, 600);
    
    // Używamy zapisanej referencji
    currentContainer.appendChild(renderer.domElement);

    // Oświetlenie i siatka pomocnicza
    const light = new THREE.DirectionalLight(0xffffff, 1);
    light.position.set(5, 10, 5);
    scene.add(light);
    scene.add(new THREE.AmbientLight(0x404040));
    scene.add(new THREE.GridHelper(10, 10));

    // Obiekt reprezentujący czujnik/długopis
    const penGeometry = new THREE.CylinderGeometry(0.1, 0.1, 1, 16);
    penGeometry.rotateX(Math.PI / 2);
    const penMaterial = new THREE.MeshPhongMaterial({ color: 0x007bff });
    const penMesh = new THREE.Mesh(penGeometry, penMaterial);
    scene.add(penMesh);

    // Linia śladu (trajektoria)
    const maxPoints = 10000;
    const lineMaterial = new THREE.LineBasicMaterial({ color: 0xff0000, linewidth: 2 });
    const lineGeometry = new THREE.BufferGeometry();
    const positions = new Float32Array(maxPoints * 3);
    lineGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    lineGeometry.setDrawRange(0, 0);
    const line = new THREE.Line(lineGeometry, lineMaterial);
    scene.add(line);

    let pointCount = 0;

    // Pętla renderowania
    let animationFrameId: number;
    const animate = () => {
      animationFrameId = requestAnimationFrame(animate);
      renderer.render(scene, camera);
    };
    animate();

    // Odbieranie danych z WebSocket
    const ws = new WebSocket('ws://localhost:8080');
    ws.onopen = () => console.log('Frontend połączony z serwerem danych ruchu');

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        
        if (data.position && data.quaternion) {
          const SCALE = 0.1;
          const renderTime = Date.now();
          const serverTime = data.timestamp ? data.timestamp * 1000 : renderTime;
          const latencyMs = renderTime - serverTime;

          setLatency(Math.max(0, latencyMs));

          const boundedLatency = Math.max(0, latencyMs);
          const samples = latencySamplesRef.current;
          samples.push(boundedLatency);
          if (samples.length > 100) {
            samples.shift();
          }

          setMaxLatency(Math.max(...samples));
          
          const timestamp = data.timestamp ? new Date(data.timestamp * 1000).toISOString() : 'N/A';
          console.log(`[${timestamp}] Opóźnienie: ${latencyMs.toFixed(2)}ms`);
          const x = data.position.x * SCALE;
          const y = data.position.y * SCALE;
          const z = data.position.z * SCALE;

          penMesh.position.set(x, y, z);
          penMesh.quaternion.set(
            data.quaternion.x,
            data.quaternion.y,
            data.quaternion.z,
            data.quaternion.w
          );

          if (pointCount < maxPoints) {
            positions[pointCount * 3] = x;
            positions[pointCount * 3 + 1] = y;
            positions[pointCount * 3 + 2] = z;
            pointCount++;
            lineGeometry.attributes.position.needsUpdate = true;
            lineGeometry.setDrawRange(0, pointCount);
          }
        }
      } catch (e) {
        console.error('Błąd dekodowania payloadu', e);
      }
    };

    // --- 5. Poprawione czyszczenie przy odmontowaniu widoku ---
    return () => {
      cancelAnimationFrame(animationFrameId);
      ws.close();
      
      // 2. Bezpieczne usuwanie konkretnego elementu canvas ze zbuforowanej referencji
      if (currentContainer && currentContainer.contains(renderer.domElement)) {
        currentContainer.removeChild(renderer.domElement);
      }
      
      // 3. Pełne sprzątanie pamięci karty graficznej
      renderer.dispose();
      penGeometry.dispose();
      penMaterial.dispose();
      lineGeometry.dispose(); // Dodano czyszczenie geometrii linii
      lineMaterial.dispose(); // Dodano czyszczenie materiału linii
    };
  }, [type]);

  return (
    <div className="canvas-container">
      <h1>Wizualizacja {type?.toUpperCase()}</h1>
      
      <div style={{
        position: 'absolute',
        top: '20px',
        right: '20px',
        background: 'rgba(0, 0, 0, 0.7)',
        color: '#00ff00',
        padding: '10px 15px',
        borderRadius: '5px',
        fontFamily: 'monospace',
        fontSize: '14px',
        zIndex: 100
      }}>
        Opóźnienie: <strong>{latency.toFixed(2)}ms</strong>
        <div>
          Największe opóźnienie (100 próbek): <strong>{maxLatency.toFixed(2)}ms</strong>
        </div>
      </div>
      
      <button
        onClick={() => navigate('/')}
        className="back-button"
      >
        ← Wróć do wyboru
      </button>
      
      {/* Jeśli 3D, podpinamy div, do którego Three.js wepnie swój canvas */}
      {type === '3d' ? (
        <div ref={containerRef} className="canvas-wrapper"></div>
      ) : (
        /* Twój stary statyczny Canvas dla wersji 2D */
        <canvas id="robot-canvas" width="800" height="600" className="canvas-wrapper"></canvas>
      )}
    </div>
  );
};

export default CanvasPage;