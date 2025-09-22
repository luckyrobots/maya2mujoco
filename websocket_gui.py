"""
Maya to MuJoCo Animation Streamer - Fixed Version
Uses frame change events instead of timers for accurate streaming
"""

import json
import time
import socket
import threading
import maya.cmds as cmds
import maya.mel as mel

# ============================================================================
# CONFIGURATION
# ============================================================================
class Config:
    """Central configuration for the streamer"""
    HOST = "127.0.0.1"
    PORT = 5000
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
        self.is_streaming = False
        self.last_sent_frame = -1
        self.frame_job = None
        self.sequential_thread = None
        self.stream_mode = "realtime"  # "realtime" or "sequential"
        
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
    """Map a value from one range to another"""
    if isinstance(value, (list, tuple)):
        value = value[0]
    
    value = float(value)
    from_min, from_max = float(from_range[0]), float(from_range[1])
    to_min, to_max = float(to_range[0]), float(to_range[1])
    
    value = max(min(value, from_max), from_min)
    
    if from_max == from_min:
        return to_min
    
    normalized = (value - from_min) / (from_max - from_min)
    return to_min + normalized * (to_max - to_min)

def send_frame_data(frame_number):
    """Send data for a specific frame"""
    if not state.socket_client:
        return False
        
    frame_data = {
        "type": "FRAME",
        "frame": int(frame_number),
        "timestamp": time.time(),
        "joints": {}
    }
    
    for mapping in state.actuator_mappings:
        try:
            # Get value at specific frame
            maya_value = cmds.getAttr(mapping["maya_attr"], time=frame_number)
            
            mujoco_value = map_value(
                maya_value,
                (mapping["maya_min"], mapping["maya_max"]),
                (mapping["mujoco_min"], mapping["mujoco_max"])
            )
            
            frame_data["joints"][mapping['mujoco_act']] = mujoco_value
            
        except Exception as e:
            print(f"Warning: Failed to read {mapping['maya_attr']}: {e}")
            continue
    
    if not frame_data["joints"]:
        return False
    
    try:
        message = (json.dumps(frame_data) + "\n").encode()
        state.socket_client.sendall(message)
        return True
    except socket.error as e:
        print(f"Connection error: {e}")
        return False

def on_frame_changed():
    """Called when Maya's frame changes (real-time mode)"""
    if not state.is_streaming or state.stream_mode != "realtime":
        return
        
    current_frame = int(cmds.currentTime(q=True))
    
    # Handle backwards scrubbing or jumping
    if state.last_sent_frame >= 0 and current_frame != state.last_sent_frame + 1:
        # Non-sequential change - just send current frame
        if send_frame_data(current_frame):
            print(f"Sent frame {current_frame} (jumped)")
            state.last_sent_frame = current_frame
    else:
        # Sequential playback
        if send_frame_data(current_frame):
            state.last_sent_frame = current_frame

def sequential_stream_thread():
    """Thread function for sequential streaming"""
    start = int(cmds.playbackOptions(q=True, minTime=True))
    end = int(cmds.playbackOptions(q=True, maxTime=True))
    
    print(f"Sequential streaming: frames {start} to {end}")
    
    # Store original frame
    original_frame = cmds.currentTime(q=True)
    
    # Calculate timing
    maya_fps = mel.eval('currentTimeUnitToFPS')
    frame_duration = 1.0 / maya_fps
    
    frames_sent = 0
    start_time = time.time()
    
    try:
        for frame in range(start, end + 1):
            if not state.is_streaming:
                break
                
            frame_start = time.time()
            
            # Move to frame and evaluate
            cmds.currentTime(frame, edit=True, update=True)
            
            # Send frame data
            if send_frame_data(frame):
                frames_sent += 1
                if frames_sent % 10 == 0:
                    print(f"Progress: {frame}/{end} ({frames_sent} sent)")
            
            # Timing control
            elapsed = time.time() - frame_start
            if elapsed < frame_duration:
                time.sleep(frame_duration - elapsed)
                
    except Exception as e:
        print(f"Sequential stream error: {e}")
        
    finally:
        # Restore original frame
        cmds.currentTime(original_frame, edit=True)
        
        total_time = time.time() - start_time
        print(f"\nSequential stream complete:")
        print(f"  Frames sent: {frames_sent}")
        print(f"  Time: {total_time:.1f}s")
        print(f"  Avg FPS: {frames_sent/total_time:.1f}")
        
        state.is_streaming = False
        cmds.evalDeferred(update_ui_after_stream)

def update_ui_after_stream():
    """Update UI after streaming completes"""
    cmds.button("realtimeBtn", e=True, label="Real-time Stream", bgc=(0.3, 0.6, 0.3))
    cmds.button("sequentialBtn", e=True, label="Sequential Stream", bgc=(0.3, 0.6, 0.6))

# ============================================================================
# STREAMING CONTROL
# ============================================================================
def start_realtime_stream(*args):
    """Start real-time streaming (follows timeline)"""
    if not state.socket_client:
        cmds.warning("Not connected!")
        return
        
    if state.is_streaming:
        stop_streaming()
        return
        
    state.stream_mode = "realtime"
    state.is_streaming = True
    state.last_sent_frame = -1
    
    # Create script job for frame changes
    if state.frame_job:
        cmds.scriptJob(kill=state.frame_job)
    state.frame_job = cmds.scriptJob(event=["timeChanged", on_frame_changed])
    
    cmds.button("realtimeBtn", e=True, label="Stop Real-time", bgc=(0.6, 0.3, 0.3))
    cmds.button("sequentialBtn", e=True, enable=False)
    
    cmds.inViewMessage(amg="Real-time streaming active", pos="topCenter", fade=True)
    print("Real-time streaming started - play timeline or scrub")

def start_sequential_stream(*args):
    """Start sequential streaming (forces all frames)"""
    if not state.socket_client:
        cmds.warning("Not connected!")
        return
        
    if state.is_streaming:
        stop_streaming()
        return
        
    state.stream_mode = "sequential"
    state.is_streaming = True
    
    # Start thread
    state.sequential_thread = threading.Thread(target=sequential_stream_thread)
    state.sequential_thread.daemon = True
    state.sequential_thread.start()
    
    cmds.button("sequentialBtn", e=True, label="Stop Sequential", bgc=(0.6, 0.3, 0.3))
    cmds.button("realtimeBtn", e=True, enable=False)
    
    cmds.inViewMessage(amg="Sequential streaming started", pos="topCenter", fade=True)

def stop_streaming(*args):
    """Stop any active streaming"""
    state.is_streaming = False
    
    # Kill script job
    if state.frame_job:
        try:
            cmds.scriptJob(kill=state.frame_job)
        except:
            pass
        state.frame_job = None
    
    # Wait for thread
    if state.sequential_thread and state.sequential_thread.is_alive():
        state.sequential_thread.join(timeout=1)
    
    update_ui_after_stream()
    cmds.inViewMessage(amg="Streaming stopped", pos="topCenter", fade=True)
    print("Streaming stopped")

# ============================================================================
# CONNECTION MANAGEMENT
# ============================================================================
def connect_mujoco(*args):
    """Establish connection to MuJoCo server"""
    if state.socket_client:
        cmds.warning("Already connected!")
        return
    
    try:
        state.socket_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        state.socket_client.connect((Config.HOST, Config.PORT))
        state.socket_client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        state.socket_client.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, Config.SOCKET_BUFFER_SIZE)
        
        cmds.button("connectBtn", e=True, label="Connected ✓", bgc=(0.2, 0.6, 0.2))
        cmds.button("realtimeBtn", e=True, enable=True)
        cmds.button("sequentialBtn", e=True, enable=True)
        
        cmds.inViewMessage(amg="Connected to MuJoCo", pos="topCenter", fade=True)
        print(f"Connected to MuJoCo at {Config.HOST}:{Config.PORT}")
        
    except Exception as e:
        state.socket_client = None
        cmds.warning(f"Connection failed: {e}")

def disconnect_mujoco(*args):
    """Disconnect from MuJoCo server"""
    stop_streaming()
    
    if state.socket_client:
        try:
            state.socket_client.close()
        except:
            pass
    
    state.reset_connection()
    
    cmds.button("connectBtn", e=True, label="Connect", bgc=(0.3, 0.5, 0.8))
    cmds.button("realtimeBtn", e=True, label="Real-time Stream", bgc=(0.3, 0.6, 0.3), enable=False)
    cmds.button("sequentialBtn", e=True, label="Sequential Stream", bgc=(0.3, 0.6, 0.6), enable=False)
    
    cmds.inViewMessage(amg="Disconnected", pos="topCenter", fade=True)
    print("Disconnected from MuJoCo")

# ============================================================================
# MAPPING MANAGEMENT (unchanged from your version)
# ============================================================================
def refresh_mapping_list():
    """Update the UI list of mappings"""
    cmds.textScrollList("mappingList", e=True, removeAll=True)
    
    for mapping in state.actuator_mappings:
        label = f"{mapping['maya_attr']} → {mapping['mujoco_act']}"
        cmds.textScrollList("mappingList", e=True, append=label)

def add_mapping(*args):
    """Add a new actuator mapping"""
    maya_attr = cmds.textField("mayaAttr", q=True, text=True).strip()
    mujoco_act = cmds.textField("mujocoAct", q=True, text=True).strip()
    
    if not maya_attr or not mujoco_act:
        cmds.warning("Both fields required!")
        return
    
    try:
        maya_min = float(cmds.floatField("mayaMin", q=True, value=True))
        maya_max = float(cmds.floatField("mayaMax", q=True, value=True))
        mujoco_min = float(cmds.floatField("mujocoMin", q=True, value=True))
        mujoco_max = float(cmds.floatField("mujocoMax", q=True, value=True))
    except:
        cmds.warning("Invalid range values!")
        return
    
    state.actuator_mappings.append({
        "maya_attr": maya_attr,
        "mujoco_act": mujoco_act,
        "maya_min": maya_min,
        "maya_max": maya_max,
        "mujoco_min": mujoco_min,
        "mujoco_max": mujoco_max
    })
    
    refresh_mapping_list()
    print(f"Added: {maya_attr} → {mujoco_act}")

def remove_mapping(*args):
    """Remove selected mapping"""
    selected = cmds.textScrollList("mappingList", q=True, selectIndexedItem=True)
    if selected:
        idx = selected[0] - 1
        removed = state.actuator_mappings.pop(idx)
        refresh_mapping_list()

def save_mappings(*args):
    """Save mappings to JSON file"""
    path = cmds.fileDialog2(fileMode=0, caption="Save", fileFilter="JSON (*.json)")
    if path:
        with open(path[0], 'w') as f:
            json.dump(state.actuator_mappings, f, indent=2)
        cmds.inViewMessage(amg="Saved", pos="topCenter", fade=True)

def load_mappings(*args):
    """Load mappings from JSON file"""
    path = cmds.fileDialog2(fileMode=1, caption="Load", fileFilter="JSON (*.json)")
    if path:
        try:
            with open(path[0], 'r') as f:
                state.actuator_mappings = json.load(f)
            refresh_mapping_list()
            cmds.inViewMessage(amg="Loaded", pos="topCenter", fade=True)
        except Exception as e:
            cmds.warning(f"Failed: {e}")

# ============================================================================
# USER INTERFACE
# ============================================================================
def create_ui():
    """Create the streamer UI"""
    window_name = "mujocoStreamer"
    
    if cmds.window(window_name, exists=True):
        cmds.deleteUI(window_name)
    
    window = cmds.window(window_name, title="MuJoCo Streamer", widthHeight=(450, 650))
    
    cmds.columnLayout(adjustableColumn=True, rowSpacing=10)
    
    # Connection
    cmds.frameLayout(label="Connection", collapsable=True)
    cmds.rowLayout(numberOfColumns=2, columnWidth2=(225, 225))
    cmds.button("connectBtn", label="Connect", command=connect_mujoco, 
                height=30, bgc=(0.3, 0.5, 0.8))
    cmds.button(label="Disconnect", command=disconnect_mujoco, 
                height=30, bgc=(0.5, 0.5, 0.5))
    cmds.setParent("..")
    cmds.setParent("..")
    
    # Streaming modes
    cmds.frameLayout(label="Streaming Mode", collapsable=True)
    cmds.columnLayout(adjustableColumn=True)
    cmds.text(label="Choose streaming mode:", font="boldLabelFont")
    cmds.separator(height=5)
    cmds.button("realtimeBtn", label="Real-time Stream", command=start_realtime_stream,
                height=35, bgc=(0.3, 0.6, 0.3), enable=False,
                annotation="Streams as you play/scrub timeline")
    cmds.button("sequentialBtn", label="Sequential Stream", command=start_sequential_stream,
                height=35, bgc=(0.3, 0.6, 0.6), enable=False,
                annotation="Forces every frame in sequence")
    cmds.text(label="Real-time: Follow timeline playback", font="smallObliqueLabelFont")
    cmds.text(label="Sequential: Guarantee all frames sent", font="smallObliqueLabelFont")
    cmds.setParent("..")
    cmds.setParent("..")
    
    # Mappings
    cmds.frameLayout(label="Joint Mappings", collapsable=True)
    
    cmds.text(label="Active Mappings:", align="left")
    cmds.textScrollList("mappingList", height=120)
    
    cmds.separator(height=10)
    
    cmds.text(label="Add Mapping:", font="boldLabelFont")
    
    cmds.rowLayout(numberOfColumns=2, columnWidth2=(120, 325))
    cmds.text(label="Maya Attr:")
    cmds.textField("mayaAttr")
    cmds.setParent("..")
    
    cmds.rowLayout(numberOfColumns=2, columnWidth2=(120, 325))
    cmds.text(label="MuJoCo Act:")
    cmds.textField("mujocoAct")
    cmds.setParent("..")
    
    cmds.rowLayout(numberOfColumns=5, columnWidth5=(80, 80, 40, 80, 165))
    cmds.text(label="Maya:")
    cmds.floatField("mayaMin", value=Config.DEFAULT_MAYA_RANGE[0], precision=1)
    cmds.text(label=" to ")
    cmds.floatField("mayaMax", value=Config.DEFAULT_MAYA_RANGE[1], precision=1)
    cmds.text(label=" degrees", font="smallPlainLabelFont")
    cmds.setParent("..")
    
    cmds.rowLayout(numberOfColumns=5, columnWidth5=(80, 80, 40, 80, 165))
    cmds.text(label="MuJoCo:")
    cmds.floatField("mujocoMin", value=Config.DEFAULT_MUJOCO_RANGE[0], precision=2)
    cmds.text(label=" to ")
    cmds.floatField("mujocoMax", value=Config.DEFAULT_MUJOCO_RANGE[1], precision=2)
    cmds.text(label=" radians", font="smallPlainLabelFont")
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