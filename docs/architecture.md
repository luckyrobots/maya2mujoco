## System architecture

### Components
- **MuJoCo server (`mujoco_server.py`)**: Loads the MuJoCo model, opens the viewer, listens on TCP, applies joint commands per incoming frame, and auto-loops the last received animation when the client disconnects.
- **Maya streamer (`maya_streamer.py`)**: Runs inside Maya, provides a small UI to connect/disconnect, define joint mappings, gather timeline data, and stream frames to the server.
- **Mapping example (`maya_to_mujoco.json`)**: Sample Maya-attribute to MuJoCo-actuator mappings and value ranges.

### Data flow
1. Start the MuJoCo server locally; it logs discovered actuators and waits for a client.
2. In Maya, open the streamer UI and connect to the server.
3. The streamer gathers frame-by-frame joint values for the current timeline based on mappings.
4. Frames are serialized as JSON lines and sent over TCP to the server.
5. The server applies controls to actuators and steps/syncs the viewer. If Maya disconnects, it loops the last cached animation at maximum speed.

### Networking
- Transport: TCP over `127.0.0.1:5000` by default.
- Messages: newline-delimited JSON with shape `{ type: "FRAME", frame: number, timestamp: number, joints: { [actuatorName]: float } }`.

### Configuration
- Server configuration lives in the `Config` class inside `mujoco_server.py` (host/port, XML path, timestep, clamp limits, buffers).
- Streamer configuration lives in the `Config` class inside `maya_streamer.py` (host/port and default ranges).

### Runbook (summary)
- Outside Maya:
  - `python -m venv .venv` then activate.
  - `pip install -r requirements.txt`.
  - `python mujoco_server.py`.
- Inside Maya:
  - Run the snippet from README to load/reload `maya_streamer.py` and open the UI.

### Repository layout (lightweight)
```
scripts/
  mujoco_server.py        # MuJoCo-side TCP server + viewer
  maya_streamer.py        # Maya-side UI and streaming
  maya_to_mujoco.json     # Example mappings
  README.md               # Usage and setup
  requirements.txt        # MuJoCo dependency for server
  LICENSE                 # MIT license
  docs/architecture.md    # This overview
```

This layout is intentionally minimal to reduce friction when running from Maya and a local Python.

