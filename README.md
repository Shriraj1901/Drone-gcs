# Drone GCS

A local, web-based Ground Control Station for ArduPilot drones. It connects to a
flight controller over MAVLink (SiK telemetry radio, UDP or TCP) and shows live
telemetry, a map, and live video in one browser dashboard.

## Features
- Connection manager: serial (COM port), UDP, TCP, or built-in simulation
- Live telemetry at 10 Hz over WebSocket
- Leaflet map with rotating drone icon, flight trail, and home marker
- HUD: altitude, speed, heading, battery, mode, GPS status
- Live video: webcam, RTSP, UDP, or test pattern
- Commands with safety checks: arm, disarm, takeoff, land, RTL, mode change
- Simulation mode (no drone needed)

## Quick start (Windows)
```powershell
python -m venv venv
.\venv\Scripts\Activate.ps1
pip install -r requirements.txt
python main.py
```
Open http://localhost:8000, choose **Simulation**, and click **Connect**.

## Hardware used
Pixhawk 2.4.8 (ArduPilot), Holybro SiK telemetry radios (57600 baud),
GPS and compass module.

## Roadmap
- [x] Phase 1: telemetry, map, HUD, video
- [x] Phase 2A: commands with safety checks
- [ ] Phase 2B: calibration
- [ ] Phase 2C: mission planning
- [ ] Phase 2D: flight logging and replay
- [ ] Phase 3: geofence, parameters, offline maps, object detection

## Safety
This software can send commands that spin motors. **Remove propellers for all
bench tests**, test in the simulator first, and always keep your RC transmitter
ready as the emergency control. Use at your own risk.

## Licence
MIT