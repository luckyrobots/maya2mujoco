"""
MuJoCo Animation Server with Automatic Looping
Automatically loops the last received animation when Maya disconnects
No FPS throttling - runs as fast as possible
"""

import socket
import threading
import time
import json
import mujoco
import mujoco.viewer

# ============================================================================
# CONFIGURATION
# ============================================================================
class Config:
    """Central configuration for the server"""
    HOST = "127.0.0.1"
    PORT = 5000
    XML_PATH = r"C:/Users/ethan/OneDrive/Documents/LuckyWorld/Robots/unitree_g1_new/scene.xml"
    
    # Simulation settings
    TIMESTEP = 0.005  # 200 Hz simulation
    JOINT_LIMIT = 3.14  # Maximum joint angle (radians)
    
    # Network settings
    BUFFER_SIZE = 8192
    SOCKET_BUFFER = 65536

    # Playback settings (server-side pacing)
    PLAYBACK_FPS = 30.0  # Target playback FPS when pacing on server
    LOOP_WHEN_COMPLETE = True  # Loop cached animation when stream ends

# ============================================================================
# SERVER STATE
# ============================================================================
class ServerState:
    """Manages the server state"""
    def __init__(self):
        self.client_socket = None
        self.receive_buffer = ""
        
        # Frame tracking
        self.total_frames = 0
        self.fps_counter = 0
        self.last_fps_time = time.time()
        self.last_frame_number = -1
        
        # Animation cache for auto-looping
        self.animation_frames = []  # List of frame data
        self.loop_index = 0
        self.is_receiving = False  # True when receiving from Maya
        
        # Server-side playback pacing
        self.playback_fps = Config.PLAYBACK_FPS
        self.playback_dt = 1.0 / self.playback_fps
        self.steps_per_frame = max(1, int(round(self.playback_dt / Config.TIMESTEP)))
        self.next_play_time = None
        self.loop_when_complete = Config.LOOP_WHEN_COMPLETE
        # Fractional step accumulator for precise average stepping
        self.step_accumulator = 0.0
        
    def reset_client(self):
        """Reset client connection state"""
        self.client_socket = None
        self.receive_buffer = ""
        self.is_receiving = False
        self.last_frame_number = -1
        # Animation cache is preserved for looping

# ============================================================================
# MUJOCO SETUP
# ============================================================================
class MuJoCoSimulation:
    """Handles MuJoCo model and simulation"""
    def __init__(self):
        print(f"Loading MuJoCo model from: {Config.XML_PATH}")
        self.model = mujoco.MjModel.from_xml_path(Config.XML_PATH)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = Config.TIMESTEP
        
        # Store initial state for reset
        self.initial_qpos = self.data.qpos.copy()
        self.initial_qvel = self.data.qvel.copy()
        self.initial_ctrl = self.data.ctrl.copy()
        
        # Build actuator lookup table
        self.actuator_map = self._build_actuator_map()
        
        # Launch viewer
        print("Starting MuJoCo viewer...")
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
        self.viewer.pause = False
        
    def _build_actuator_map(self):
        """Create actuator name to ID mapping"""
        actuator_map = {}
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if name:
                actuator_map[name] = i
                print(f"  Actuator: {name} (ID: {i})")
        print(f"Total actuators: {len(actuator_map)}")
        return actuator_map
    
    def reset_to_start(self):
        """Reset simulation to initial state"""
        self.data.qpos[:] = self.initial_qpos
        self.data.qvel[:] = self.initial_qvel
        self.data.ctrl[:] = self.initial_ctrl
        mujoco.mj_forward(self.model, self.data)
    
    def apply_controls(self, joint_values):
        """Apply joint values to the simulation"""
        applied = 0
        for name, value in joint_values.items():
            actuator_id = self.actuator_map.get(name)
            if actuator_id is not None:
                # Clamp value to joint limits
                clamped = max(-Config.JOINT_LIMIT, min(Config.JOINT_LIMIT, float(value)))
                self.data.ctrl[actuator_id] = clamped
                applied += 1
        return applied
    
    def step(self):
        """Step the simulation forward"""
        mujoco.mj_step(self.model, self.data)
        self.viewer.sync()
    
    def is_running(self):
        """Check if viewer is still running"""
        return self.viewer.is_running()

# ============================================================================
# NETWORK SERVER
# ============================================================================
class AnimationServer:
    """Handles network communication"""
    def __init__(self):
        self.state = ServerState()
        self.server_socket = self._create_server()
        self.running = True
        
    def _create_server(self):
        """Create and configure server socket"""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((Config.HOST, Config.PORT))
        server.listen(1)
        server.setblocking(False)
        print(f"\nServer listening on {Config.HOST}:{Config.PORT}")
        return server
    
    def accept_clients(self):
        """Background thread to accept client connections"""
        while self.running:
            if self.state.client_socket is None:
                try:
                    client, addr = self.server_socket.accept()
                    print(f"\n✓ Maya connected from {addr}")
                    print("Receiving animation...")
                    
                    # Configure client socket
                    client.setblocking(False)
                    client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    client.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, Config.SOCKET_BUFFER)
                    
                    self.state.client_socket = client
                    self.state.receive_buffer = ""
                    self.state.is_receiving = True
                    
                except BlockingIOError:
                    pass
                except Exception as e:
                    print(f"Connection error: {e}")
            
            time.sleep(0.01)
    
    def receive_data(self):
        """Receive data from client"""
        if not self.state.client_socket:
            return None
        
        try:
            chunk = self.state.client_socket.recv(Config.BUFFER_SIZE).decode()
            if chunk:
                self.state.receive_buffer += chunk
                return True
            return False
            
        except BlockingIOError:
            return False
        except ConnectionResetError:
            print("\n✗ Maya disconnected")
            print(f"Animation cached: {len(self.state.animation_frames)} frames")
            print(f"Playing loop at {self.state.playback_fps:.1f} FPS...")
            self.state.reset_client()
            return None
        except Exception as e:
            print(f"Receive error: {e}")
            return False
    
    def get_next_message(self):
        """Get next complete message from buffer"""
        if "\n" in self.state.receive_buffer:
            line, self.state.receive_buffer = self.state.receive_buffer.split("\n", 1)
            return line.strip()
        return None
    
    def close(self):
        """Clean up server"""
        self.running = False
        if self.state.client_socket:
            self.state.client_socket.close()
        self.server_socket.close()

# ============================================================================
# ANIMATION PROCESSING & PLAYBACK PACING
# ============================================================================
def set_playback_fps(state, fps):
    """Update server-side playback FPS and recompute pacing."""
    try:
        new_fps = float(fps)
        if new_fps <= 0:
            return
    except Exception:
        return
    state.playback_fps = new_fps
    state.playback_dt = 1.0 / new_fps
    # Compute exact required substeps per displayed frame
    exact_steps = state.playback_dt / Config.TIMESTEP
    state.steps_per_frame = max(0, int(exact_steps))
    state.step_accumulator = exact_steps - state.steps_per_frame
    state.next_play_time = None  # reset scheduler
    print(f"Playback FPS set to {new_fps:.1f} (steps/frame: {state.steps_per_frame})")
def process_message(message, simulation, state):
    """Process a received message and cache frame data"""
    
    try:
        payload = json.loads(message)
    except json.JSONDecodeError:
        return None
    
    # Handle animation frame
    if payload.get("type") == "FRAME":
        frame_number = payload.get("frame", -1)
        joints = payload.get("joints", {})
        incoming_fps = payload.get("fps")
        if incoming_fps is not None:
            set_playback_fps(state, incoming_fps)
        
        # Detect new animation sequence
        if frame_number < state.last_frame_number and state.last_frame_number > 0:
            print(f"New animation sequence detected (frame {frame_number})")
            state.animation_frames.clear()
            state.loop_index = 0
            state.next_play_time = None  # restart pacing
        
        # Cache the frame
        frame_data = {
            "frame": frame_number,
            "joints": joints
        }
        state.animation_frames.append(frame_data)
        
        # Check for dropped frames
        if state.last_frame_number >= 0 and frame_number > state.last_frame_number + 1:
            dropped = frame_number - state.last_frame_number - 1
            print(f"⚠ Dropped {dropped} frames")
        
        state.last_frame_number = frame_number
        
        return {"type": "frame", "applied": None, "number": frame_number}
    
    return None

def update_fps_stats(state):
    """Update and display FPS statistics"""
    current_time = time.time()
    elapsed = current_time - state.last_fps_time
    
    if elapsed >= 1.0:
        fps = state.fps_counter / elapsed
        mode = "RECEIVING" if state.is_receiving else "LOOPING"
        if state.animation_frames:
            print(f"{mode}: {fps:.1f} FPS | Frame {state.loop_index}/{len(state.animation_frames)}")
        state.fps_counter = 0
        state.last_fps_time = current_time

def play_looped_animation(simulation, state):
    """Throttled playback of cached animation at server-side FPS."""
    if not state.animation_frames:
        return False
    
    now = time.time()
    if state.next_play_time is None:
        state.next_play_time = now
    
    # Wait until the next scheduled frame time
    if now < state.next_play_time:
        return False
    
    # If we're receiving and reached the current end, wait for more frames
    if state.loop_index >= len(state.animation_frames):
        if state.is_receiving:
            state.next_play_time = now + state.playback_dt
            return False
        # End of cached clip; either loop or hold on last frame
        if state.loop_when_complete:
            state.loop_index = 0
            simulation.reset_to_start()
        else:
            state.loop_index = len(state.animation_frames) - 1
    
    # Apply current frame
    frame_data = state.animation_frames[state.loop_index]
    simulation.apply_controls(frame_data["joints"])
    
    # Step simulation for the duration of a single frame
    steps = state.steps_per_frame
    # Use the accumulator to distribute fractional steps over time
    state.step_accumulator += (state.playback_dt / Config.TIMESTEP) - steps
    if state.step_accumulator >= 1.0:
        steps += 1
        state.step_accumulator -= 1.0
    # Batch substeps and sync viewer once to avoid extra overhead
    for _ in range(steps):
        mujoco.mj_step(simulation.model, simulation.data)
    simulation.viewer.sync()
    
    # Advance index and counters
    state.loop_index += 1
    state.fps_counter += 1
    
    # Schedule next frame relative to the previous target to avoid drift
    if state.next_play_time is None:
        state.next_play_time = now + state.playback_dt
    else:
        state.next_play_time += state.playback_dt
    # If we fell behind significantly, realign to avoid burst catch-up
    if now - state.next_play_time > state.playback_dt * 0.5:
        state.next_play_time = now + state.playback_dt
    
    return True

# ============================================================================
# MAIN SIMULATION LOOP
# ============================================================================
def simulation_loop(server, simulation):
    """Main simulation and network processing loop"""
    state = server.state
    
    while simulation.is_running():
        # Process incoming data from Maya
        if server.receive_data():
            state.is_receiving = True
            # Drain all complete messages
            while True:
                message = server.get_next_message()
                if not message:
                    state.is_receiving = False
                    break
                process_message(message, simulation, state)
        
        # Pacing-controlled playback (works both while receiving and after)
        played = play_looped_animation(simulation, state)
        update_fps_stats(state)
        
        # If nothing to play yet, keep the viewer responsive
        if not played and not state.animation_frames:
            simulation.step()
        
        # Very small delay to prevent CPU lock
        time.sleep(0.0005)

# ============================================================================
# MAIN EXECUTION
# ============================================================================
def print_banner():
    """Print startup banner"""
    print("\n" + "="*60)
    print("MuJoCo Animation Server")
    print("="*60)
    
def print_instructions():
    """Print usage instructions"""
    print("\n📋 How it works:")
    print("1. Receives animation from Maya")
    print("2. Caches all frames as they arrive")
    print("3. Loops animation at maximum speed when Maya disconnects")
    print("4. Continues looping until new animation is received")
    print("\n⚠ Note: Loop playback runs as fast as possible!")
    print("\n⌨ Viewer Controls:")
    print("• Space: Pause/resume simulation")
    print("• Backspace: Reset view")
    print("• Mouse: Rotate view")
    print("• Scroll: Zoom")
    print("\n" + "-"*60)

def main():
    """Main server entry point"""
    print_banner()
    
    # Initialize components
    try:
        simulation = MuJoCoSimulation()
        server = AnimationServer()
    except Exception as e:
        print(f"❌ Initialization failed: {e}")
        return
    
    print_instructions()
    
    # Start client accept thread
    accept_thread = threading.Thread(target=server.accept_clients, daemon=True)
    accept_thread.start()
    
    print("✓ Server ready - waiting for Maya connection...\n")
    
    # Run main simulation loop
    try:
        simulation_loop(server, simulation)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        server.close()
        print("Server closed")

if __name__ == "__main__":
    main()