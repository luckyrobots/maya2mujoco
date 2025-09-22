import json
import time
import socket
import maya.cmds as cmds
import maya.api.OpenMaya as om

# ============================================================================
# CONFIGURATION
# ============================================================================
class Config:
    """Central configuration for the streamer"""
    HOST = "127.0.0.1"
    PORT = 5000
    DEFAULT_FPS = 30
    DEFAULT_MAYA_RANGE = (0, 90)      # degrees
    DEFAULT_MUJOCO_RANGE = (-3.14, 3.14)  # radians
    SOCKET_BUFFER_SIZE = 65536

# ============================================================================
# GLOBAL STATE
# ============================================================================
class StreamerState:
    """Manages the global state of the streamer"""
    def __init__(self):
        self.actuator_mappings = []
        self.socket_client = None
        self.timer_callback_id = None
        self.is_streaming = False
        self.target_fps = Config.DEFAULT_FPS
        self.last_sent_frame = -1
    
    def reset_connection(self):
        """Clean up connection state"""
        self.socket_client = None
        self.is_streaming = False
        self.last_sent_frame = -1
        
state = StreamerState()

# ============================================================================
# CORE FUNCTIONALITY
# ============================================================================
def map_value(value, from_range, to_range):
    """
    Map a value from one range to another
    Args:
        value: Input value (handles lists/tuples)
        from_range: (min, max) of input range
        to_range: (min, max) of output range
    Returns:
        Mapped value
    """
    # Handle Maya's list/tuple returns
    if isinstance(value, (list, tuple)):
        value = value[0]
    
    value = float(value)
    from_min, from_max = float(from_range[0]), float(from_range[1])
    to_min, to_max = float(to_range[0]), float(to_range[1])
    
    # Clamp to input range
    value = max(min(value, from_max), from_min)
    
    # Avoid division by zero
    if from_max == from_min:
        return to_min
    
    # Linear interpolation
    normalized = (value - from_min) / (from_max - from_min)
    return to_min + normalized * (to_max - to_min)

def build_frame_data():
    """
    Build a frame of animation data from current Maya state
    Returns:
        Dictionary with frame data or None if no mappings
    """
    current_frame = cmds.currentTime(query=True)
    
    frame_data = {
        "type": "FRAME",
        "frame": int(current_frame),
        "timestamp": time.time(),
        "joints": {}
    }
    
    for mapping in state.actuator_mappings:
        try:
            # Get Maya attribute value
            maya_value = cmds.getAttr(mapping["maya_attr"])
            
            # Map to MuJoCo range
            mujoco_value = map_value(
                maya_value,
                (mapping["maya_min"], mapping["maya_max"]),
                (mapping["mujoco_min"], mapping["mujoco_max"])
            )
            
            frame_data["joints"][mapping['mujoco_act']] = mujoco_value
            
        except Exception as e:
            print(f"Warning: Failed to read {mapping['maya_attr']}: {e}")
            continue
    
    return frame_data if frame_data["joints"] else None

def send_frame():
    """Send current frame data to MuJoCo"""
    if not state.socket_client or not state.is_streaming:
        return
    
    # Avoid sending duplicate frames
    current_frame = cmds.currentTime(query=True)
    if current_frame == state.last_sent_frame:
        return
    
    state.last_sent_frame = current_frame
    
    try:
        frame_data = build_frame_data()
        if frame_data:
            message = (json.dumps(frame_data) + "\n").encode()
            state.socket_client.sendall(message)
            print(f"Frame {int(current_frame)}: Sent {len(frame_data['joints'])} joints")
    except socket.error as e:
        print(f"Connection error: {e}")
        cmds.warning("Lost connection to MuJoCo!")
        disconnect_mujoco()
    except Exception as e:
        print(f"Failed to send frame: {e}")

# ============================================================================
# TIMER MANAGEMENT
# ============================================================================
def start_streaming_timer():
    """Start the Maya timer for streaming"""
    stop_streaming_timer()  # Clean up any existing timer
    
    try:
        interval = max(0.001, 1.0 / state.target_fps)
        
        def timer_callback(elapsed_time, last_time, client_data):
            if state.is_streaming and state.socket_client:
                send_frame()
        
        state.timer_callback_id = om.MTimerMessage.addTimerCallback(
            interval, timer_callback
        )
        print(f"Streaming timer started at {state.target_fps} FPS")
    except Exception as e:
        print(f"Failed to start timer: {e}")

def stop_streaming_timer():
    """Stop the Maya timer"""
    if state.timer_callback_id:
        try:
            om.MMessage.removeCallback(state.timer_callback_id)
        except:
            pass
        state.timer_callback_id = None

# ============================================================================
# CONNECTION MANAGEMENT
# ============================================================================
def connect_mujoco(*args):
    """Establish connection to MuJoCo server"""
    if state.socket_client:
        cmds.warning("Already connected!")
        return
    
    try:
        # Create and configure socket
        state.socket_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        state.socket_client.connect((Config.HOST, Config.PORT))
        state.socket_client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        state.socket_client.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, Config.SOCKET_BUFFER_SIZE)
        
        # Update UI
        cmds.button("connectBtn", e=True, label="Connected ✓", bgc=(0.2, 0.6, 0.2))
        cmds.button("streamBtn", e=True, enable=True)
        
        cmds.inViewMessage(amg="Connected to MuJoCo", pos="topCenter", fade=True)
        print(f"Connected to MuJoCo at {Config.HOST}:{Config.PORT}")
        
    except Exception as e:
        state.socket_client = None
        cmds.warning(f"Connection failed: {e}")

def disconnect_mujoco(*args):
    """Disconnect from MuJoCo server"""
    # Stop streaming first
    if state.is_streaming:
        toggle_streaming()
    
    # Close socket
    if state.socket_client:
        try:
            state.socket_client.close()
        except:
            pass
    
    state.reset_connection()
    stop_streaming_timer()
    
    # Update UI
    cmds.button("connectBtn", e=True, label="Connect", bgc=(0.3, 0.5, 0.8))
    cmds.button("streamBtn", e=True, label="Start Streaming", bgc=(0.3, 0.6, 0.3), enable=False)
    
    cmds.inViewMessage(amg="Disconnected", pos="topCenter", fade=True)
    print("Disconnected from MuJoCo")

def toggle_streaming(*args):
    """Toggle animation streaming on/off"""
    if not state.socket_client:
        cmds.warning("Not connected to MuJoCo!")
        return
    
    state.is_streaming = not state.is_streaming
    
    if state.is_streaming:
        state.last_sent_frame = -1
        start_streaming_timer()
        cmds.button("streamBtn", e=True, label="Stop Streaming", bgc=(0.6, 0.3, 0.3))
        cmds.inViewMessage(amg="Streaming started", pos="topCenter", fade=True)
    else:
        stop_streaming_timer()
        cmds.button("streamBtn", e=True, label="Start Streaming", bgc=(0.3, 0.6, 0.3))
        cmds.inViewMessage(amg="Streaming stopped", pos="topCenter", fade=True)

# ============================================================================
# MAPPING MANAGEMENT
# ============================================================================
def refresh_mapping_list():
    """Update the UI list of mappings"""
    cmds.textScrollList("mappingList", e=True, removeAll=True)
    
    for mapping in state.actuator_mappings:
        label = f"{mapping['maya_attr']} → {mapping['mujoco_act']}"
        cmds.textScrollList("mappingList", e=True, append=label)

def add_mapping(*args):
    """Add a new actuator mapping"""
    # Get values from UI
    maya_attr = cmds.textField("mayaAttr", q=True, text=True).strip()
    mujoco_act = cmds.textField("mujocoAct", q=True, text=True).strip()
    
    if not maya_attr or not mujoco_act:
        cmds.warning("Both Maya attribute and MuJoCo actuator must be specified!")
        return
    
    try:
        maya_min = float(cmds.floatField("mayaMin", q=True, value=True))
        maya_max = float(cmds.floatField("mayaMax", q=True, value=True))
        mujoco_min = float(cmds.floatField("mujocoMin", q=True, value=True))
        mujoco_max = float(cmds.floatField("mujocoMax", q=True, value=True))
    except:
        cmds.warning("Invalid range values!")
        return
    
    # Add mapping
    state.actuator_mappings.append({
        "maya_attr": maya_attr,
        "mujoco_act": mujoco_act,
        "maya_min": maya_min,
        "maya_max": maya_max,
        "mujoco_min": mujoco_min,
        "mujoco_max": mujoco_max
    })
    
    refresh_mapping_list()
    print(f"Added mapping: {maya_attr} → {mujoco_act}")

def remove_mapping(*args):
    """Remove selected mapping"""
    selected = cmds.textScrollList("mappingList", q=True, selectIndexedItem=True)
    if selected:
        idx = selected[0] - 1
        removed = state.actuator_mappings.pop(idx)
        refresh_mapping_list()
        print(f"Removed mapping: {removed['maya_attr']} → {removed['mujoco_act']}")

def save_mappings(*args):
    """Save mappings to JSON file"""
    path = cmds.fileDialog2(fileMode=0, caption="Save Mappings", fileFilter="JSON (*.json)")
    if path:
        with open(path[0], 'w') as f:
            json.dump(state.actuator_mappings, f, indent=2)
        cmds.inViewMessage(amg=f"Saved mappings", pos="topCenter", fade=True)

def load_mappings(*args):
    """Load mappings from JSON file"""
    path = cmds.fileDialog2(fileMode=1, caption="Load Mappings", fileFilter="JSON (*.json)")
    if path:
        try:
            with open(path[0], 'r') as f:
                state.actuator_mappings = json.load(f)
            refresh_mapping_list()
            cmds.inViewMessage(amg=f"Loaded mappings", pos="topCenter", fade=True)
        except Exception as e:
            cmds.warning(f"Failed to load: {e}")

def update_fps(*args):
    """Update target FPS from UI"""
    try:
        new_fps = float(cmds.intField("fpsField", q=True, value=True))
        if new_fps > 0:
            state.target_fps = new_fps
            if state.is_streaming:
                start_streaming_timer()  # Restart with new FPS
    except:
        pass

# ============================================================================
# USER INTERFACE
# ============================================================================
def create_ui():
    """Create the streamer UI"""
    window_name = "mujocoStreamer"
    
    if cmds.window(window_name, exists=True):
        cmds.deleteUI(window_name)
    
    window = cmds.window(window_name, title="MuJoCo Streamer", widthHeight=(450, 600))
    
    main_layout = cmds.columnLayout(adjustableColumn=True, rowSpacing=10)
    
    # === Connection Section ===
    cmds.frameLayout(label="Connection", collapsable=True, collapse=False)
    cmds.rowLayout(numberOfColumns=2, columnWidth2=(225, 225))
    cmds.button("connectBtn", label="Connect", command=connect_mujoco, 
                height=30, bgc=(0.3, 0.5, 0.8))
    cmds.button(label="Disconnect", command=disconnect_mujoco, 
                height=30, bgc=(0.5, 0.5, 0.5))
    cmds.setParent("..")
    cmds.setParent("..")
    
    # === Streaming Control ===
    cmds.frameLayout(label="Streaming", collapsable=True, collapse=False)
    cmds.rowLayout(numberOfColumns=3, columnWidth3=(200, 150, 100))
    cmds.button("streamBtn", label="Start Streaming", command=toggle_streaming,
                height=30, bgc=(0.3, 0.6, 0.3), enable=False)
    cmds.text(label="   Target FPS:", align="right")
    cmds.intField("fpsField", value=state.target_fps, minValue=1, maxValue=120, 
                  changeCommand=update_fps, width=50)
    cmds.setParent("..")
    cmds.setParent("..")
    
    # === Mappings Section ===
    cmds.frameLayout(label="Joint Mappings", collapsable=True, collapse=False)
    
    # Mapping list
    cmds.text(label="Active Mappings:", align="left")
    cmds.textScrollList("mappingList", height=150)
    
    cmds.separator(height=10)
    
    # Add new mapping
    cmds.text(label="Add New Mapping:", align="left", font="boldLabelFont")
    
    cmds.rowLayout(numberOfColumns=2, columnWidth2=(150, 300))
    cmds.text(label="Maya Attribute:")
    cmds.textField("mayaAttr", placeholderText="e.g., joint1.rotateX")
    cmds.setParent("..")
    
    cmds.rowLayout(numberOfColumns=2, columnWidth2=(150, 300))
    cmds.text(label="MuJoCo Actuator:")
    cmds.textField("mujocoAct", placeholderText="e.g., shoulder_joint")
    cmds.setParent("..")
    
    # Range settings
    cmds.rowLayout(numberOfColumns=5, columnWidth5=(100, 80, 50, 80, 140))
    cmds.text(label="Maya Range:")
    cmds.floatField("mayaMin", value=Config.DEFAULT_MAYA_RANGE[0], precision=1)
    cmds.text(label=" to ")
    cmds.floatField("mayaMax", value=Config.DEFAULT_MAYA_RANGE[1], precision=1)
    cmds.text(label=" (degrees)", font="smallPlainLabelFont")
    cmds.setParent("..")
    
    cmds.rowLayout(numberOfColumns=5, columnWidth5=(100, 80, 50, 80, 140))
    cmds.text(label="MuJoCo Range:")
    cmds.floatField("mujocoMin", value=Config.DEFAULT_MUJOCO_RANGE[0], precision=2)
    cmds.text(label=" to ")
    cmds.floatField("mujocoMax", value=Config.DEFAULT_MUJOCO_RANGE[1], precision=2)
    cmds.text(label=" (radians)", font="smallPlainLabelFont")
    cmds.setParent("..")
    
    cmds.separator(height=10)
    
    # Mapping buttons
    cmds.rowLayout(numberOfColumns=4, columnWidth4=(110, 110, 110, 110))
    cmds.button(label="Add", command=add_mapping, bgc=(0.3, 0.6, 0.3))
    cmds.button(label="Remove", command=remove_mapping, bgc=(0.6, 0.3, 0.3))
    cmds.button(label="Save", command=save_mappings, bgc=(0.3, 0.3, 0.6))
    cmds.button(label="Load", command=load_mappings, bgc=(0.6, 0.6, 0.3))
    cmds.setParent("..")
    cmds.setParent("..")
    
    # === Instructions ===
    cmds.frameLayout(label="Instructions", collapsable=True, collapse=True)
    cmds.text(label="1. Connect to MuJoCo server", align="left")
    cmds.text(label="2. Add joint mappings (Maya attributes → MuJoCo actuators)", align="left")
    cmds.text(label="3. Start streaming", align="left")
    cmds.text(label="4. Play animation in Maya timeline", align="left")
    cmds.setParent("..")
    
    cmds.showWindow(window)
    refresh_mapping_list()

# ============================================================================
# MAIN EXECUTION
# ============================================================================
create_ui()
print("=" * 50)
print("MuJoCo Animation Streamer Loaded")
print(f"Server: {Config.HOST}:{Config.PORT}")
print("=" * 50)