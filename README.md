# robotics
Project for robotics for human health and performence

# Running the frontend
1. Navigate to the `frontend` folder:
   ```bash
   cd frontend
   ```
2. Install dependencies:
   ```bash
   npm install
   ```
3. Install additional routing library:
   ```bash
   npm install react-router-dom
   ```
4. Start the development server with Electron:
   ```bash
   npm run dev
   ```
   - This will start the Vite dev server on `http://localhost:5173`
   - The Electron application will open automatically and load the app
5. The frontend allows you to choose between 2D and 3D visualization modes

# Running the backend
1. Create and activate **Python 3.10!!!** virtual environment in this repository
2. Clone the [FP-SNS-DATALOG1](https://github.com/STMicroelectronics/fp-sns-datalog1) repository
3. Navigate to `fp-sns-datalog1/Utilities/HSDPython_SDK`
4. Run `HSDPython_install_noGUI` script
5. Run `main.py`
