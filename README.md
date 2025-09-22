# MuJoCo <-> Maya Live Animation Streaming

This project provides a simple live streaming bridge between Autodesk Maya and a local MuJoCo simulation. A Python server receives animation frames from Maya over TCP, applies the mapped joint values to a MuJoCo model, and displays them in the MuJoCo viewer in real time.

## Overview

- `mujoco_server.py`: MuJoCo-side animation server and viewer loop. Listens for frames and applies controls to actuators.
 - `websocket_gui.py`: Maya-side streaming tool (to be executed inside Maya's Python environment). Provides a UI to connect, define joint mappings, and stream frames.
- `maya_to_mujoco.json`: Example joint mapping file (Maya attributes to MuJoCo actuator names and ranges).

Data flow:
1) Start the MuJoCo server locally and open the viewer.
2) In Maya, run `websocket_gui.py` to open the UI, connect to the server, configure mappings, and start streaming.
3) Scrub/play the Maya timeline to stream frames; the MuJoCo viewer animates accordingly.

## Prerequisites

- Windows 10/11 (tested), Python 3.10+ recommended
- MuJoCo Python bindings installed (`mujoco`), and a valid MuJoCo XML model
- Autodesk Maya with Python access to `maya.cmds` and `maya.api.OpenMaya`
- Network access to `127.0.0.1:5000` (default)

Recommended Python setup (outside Maya):
```bash
python -m venv .venv
.\.venv\Scripts\activate
pip install mujoco
```

## Configuration

Server configuration lives in each server's `Config` class:

- `HOST`/`PORT`: default `127.0.0.1:5000`
- `XML_PATH`: absolute path to your MuJoCo model XML
- `TIMESTEP`: simulation step size (e.g., 0.005 = 200 Hz)
- `JOINT_LIMIT`: clamp range for actuator commands (radians)
- `ANIMATION_TIMEOUT`: stop streaming if no frames arrive
- In `server.py`: `FRAME_PROCESS_MODE` can be `"per_frame"` or `"latest"`.

Update `XML_PATH` to point to your local model. Example from the repo:

```python
XML_PATH = r"C:/Users/ethan/OneDrive/Documents/LuckyWorld/Robots/unitree_g1_new/scene.xml"
```

## Running the MuJoCo Server

From a regular Windows PowerShell (not Maya):

```bash
cd "C:\Users\ethan\OneDrive\Documents\maya\2026\prefs\scripts"
python mujoco_server.py
```

You should see a MuJoCo viewer window and a console message:

- "Server listening on 127.0.0.1:5000"
- "✓ Server ready - waiting for Maya connection..."

Alternative: `server.py` provides a similar server with a `FRAME_PROCESS_MODE` option:

```bash
python server.py
```

Frame modes (in `server.py`):
- `per_frame`: step the simulation once for every received frame (no frame drops, may lag if Maya sends faster than the sim can step)
- `latest`: process all incoming messages but step once per tick using the latest frame (drops older frames, lower latency)

## Using the Maya Streamer UI

Execute `websocket_gui.py` inside Maya's Script Editor (Python tab):

```python
import importlib
import websocket_gui
importlib.reload(websocket_gui)
```

This opens the "MuJoCo Streamer" window with:
- Connect/Disconnect buttons
- Start/Stop Streaming and FPS control
- A Joint Mappings panel to define mappings

### Connection
1) Ensure the MuJoCo server is running
2) In the UI, click "Connect" (defaults to `127.0.0.1:5000`)
3) When connected, the Stream button is enabled

### Joint Mappings
Each mapping links a Maya attribute to a MuJoCo actuator with value ranges:
- `Maya Attribute` (e.g., `joint1.rotateX`)
- `MuJoCo Actuator` (must match the actuator name in the MuJoCo model)
- `Maya Range` (degrees)
- `MuJoCo Range` (radians)

Use the UI to Add/Remove mappings. You can Save/Load JSON mapping files. An example mapping file is included: `maya_to_mujoco.json`.

### Streaming
1) Click "Start Streaming"
2) Play or scrub the Maya timeline
3) Watch the MuJoCo viewer animate in real time

## Tips & Troubleshooting

- If the server window prints "Dropped N frames", Maya is advancing faster than the sim steps; consider `latest` mode in `server.py`.
- If no motion appears:
  - Verify actuator names printed on server startup match your mapping entries
  - Check `XML_PATH` and that the viewer opened successfully
  - Confirm the UI shows "Connected ✓" and streaming is started
- Network errors in Maya will auto-disconnect; click Connect again after the server is running
- Viewer controls: Space (pause), Backspace (reset), mouse to rotate, scroll to zoom

## File Reference

- `mujoco_server.py`: server + viewer loop, applies controls per frame and manages FPS stats
- `websocket_gui.py`: Maya UI for connection, mappings, and streaming; uses `maya.cmds` and `maya.api.OpenMaya`
- `maya_to_mujoco.json`: sample mappings for a Unitree G1 rig

## Notes

- Run servers with system Python; run the UI inside Maya's Python.
- Ranges: Maya is specified in degrees; MuJoCo expects radians.
- Actuator names must exactly match those in the MuJoCo XML. The server logs all discovered actuators on startup.

---

Maintainer notes: The project was tested on Windows with MuJoCo's Python viewer. Adjust paths and environment as needed for other platforms.


