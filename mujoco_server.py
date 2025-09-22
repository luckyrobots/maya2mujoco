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
    
    # Animation settings
    ANIMATION_TIMEOUT = 2.0  # Stop animation after 2 seconds of no data

# ============================================================================
# SERVER STATE
# ============================================================================
class ServerState:
    """Manages the server state"""
    def __init__(self):
        self.client_socket = None
        self.receive_buffer = ""
        self.is_animating = False
        
        # Frame tracking
        self.total_frames = 0
        self.fps_counter = 0
        self.last_fps_time = time.time()
        self.last_frame_number = -1
        
    def reset_client(self):
        """Reset client connection state"""
        self.client_socket = None
        self.receive_buffer = ""
        self.is_animating = False
        self.last_frame_number = -1

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
    
    def get_state(self):
        """Get current joint states"""
        state = {}
        for name, idx in self.actuator_map.items():
            state[name] = float(self.data.ctrl[idx])
        return state
    
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
                    
                    # Configure client socket
                    client.setblocking(False)
                    client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    client.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, Config.SOCKET_BUFFER)
                    
                    self.state.client_socket = client
                    self.state.receive_buffer = ""
                    
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
    
    def send_state(self, state):
        """Send current state to client"""
        if self.state.client_socket:
            try:
                message = json.dumps(state) + "\n"
                self.state.client_socket.sendall(message.encode())
                return True
            except Exception as e:
                print(f"Send error: {e}")
        return False
    
    def close(self):
        """Clean up server"""
        self.running = False
        if self.state.client_socket:
            self.state.client_socket.close()
        self.server_socket.close()

# ============================================================================
# MESSAGE PROCESSING
# ============================================================================
def process_message(message, simulation, state):
    """Process a received message"""
    
    # Handle state request
    if message == "GET_STATE":
        return {"type": "state", "data": simulation.get_state()}
    
    # Parse JSON message
    try:
        payload = json.loads(message)
    except json.JSONDecodeError:
        return None
    
    # Handle animation frame
    if payload.get("type") == "FRAME":
        frame_number = payload.get("frame", -1)
        joints = payload.get("joints", {})
        
        # Check for dropped frames
        if state.last_frame_number >= 0 and frame_number > state.last_frame_number + 1:
            dropped = frame_number - state.last_frame_number - 1
            print(f"⚠ Dropped {dropped} frames")
        
        state.last_frame_number = frame_number
        
        # Apply joint values
        applied = simulation.apply_controls(joints)
        
        # Update statistics
        state.total_frames += 1
        state.fps_counter += 1
        
        return {"type": "frame", "applied": applied, "number": frame_number}
    
    return None

def update_fps_stats(state):
    """Update and display FPS statistics"""
    current_time = time.time()
    elapsed = current_time - state.last_fps_time
    
    if elapsed >= 1.0:
        fps = state.fps_counter / elapsed
        print(f"FPS: {fps:.1f} | Frame #{state.last_frame_number} | Total: {state.total_frames}")
        state.fps_counter = 0
        state.last_fps_time = current_time

# ============================================================================
# MAIN SIMULATION LOOP
# ============================================================================
def simulation_loop(server, simulation):
    """Main simulation and network processing loop"""
    state = server.state
    
    while simulation.is_running():
        # --- Step 1: Process all available network data ---
        # Receive data and put messages in a queue
        messages = []
        if server.receive_data():
            while True:
                message = server.get_next_message()
                if not message:
                    break
                messages.append(message)

        # --- Step 2: Update simulation state from messages ---
        # Apply the LAST valid frame message from the batch.
        # This prevents the simulation from "catching up" with multiple steps
        # and instead just jumps to the latest pose.
        for message in messages:
            result = process_message(message, simulation, state)
            if result and result["type"] == "frame":
                state.is_animating = True
                update_fps_stats(state) # Update stats for each processed frame
            elif result and result["type"] == "state":
                server.send_state(result["data"])
        
        # --- Step 3: Advance the physics simulation by one step ---
        # This runs at a constant rate, regardless of how many frames arrived.
        simulation.step()

        # The viewer.sync() already handles timing, so a sleep is often not needed,
        # but a very small one can yield time to other processes.
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
    print("\n📋 Instructions:")
    print("1. Start Maya and load your scene")
    print("2. Run the Maya streamer script")
    print("3. Connect to this server")
    print("4. Configure joint mappings")
    print("5. Start streaming and play animation")
    print("\n⌨ Viewer Controls:")
    print("• Space: Pause/resume")
    print("• Backspace: Reset")
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