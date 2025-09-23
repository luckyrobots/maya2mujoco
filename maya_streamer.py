"""
Maya to MuJoCo Animation Streamer
Streams the entire animation timeline sequentially to a MuJoCo server.
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
        self.stream_thread = None
        
    def reset_connection(self):
        """Clean up connection state"""
        self.socket_client = None
        self.is_streaming = False

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
    
    # Clamp value to the source range
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
            # Get value at the specified frame
            maya_value = cmds.getAttr(mapping["maya_attr"], time=frame_number)
            
            mujoco_value = map_value(
                maya_value,
                (mapping["maya_min"], mapping["maya_max"]),
                (mapping["mujoco_min"], mapping["mujoco_max"])
            )
            
            frame_data["joints"][mapping['mujoco_act']] = mujoco_value
            
        except Exception as e:
            print(f"Warning: Failed to read {mapping['maya_attr']} at frame {frame_number}: {e}")
            continue
    
    if not frame_data["joints"]:
        return False
    
    try:
        message = (json.dumps(frame_data) + "\n").encode()
        state.socket_client.sendall(message)
        return True
    except socket.error as e:
        print(f"Connection error while sending frame {frame_number}: {e}")
        return False

def stream_thread():
    """Thread function for streaming all frames sequentially"""
    try:
        start_frame = int(cmds.playbackOptions(q=True, minTime=True))
        end_frame = int(cmds.playbackOptions(q=True, maxTime=True))
    except TypeError:
        print(f"Error getting playback options due to Maya animation not running")
        return
    
    print(f"Sequential streaming started for frames {start_frame} to {end_frame}")
    
    original_frame = cmds.currentTime(q=True)
    
    frames_sent = 0
    start_time = time.time()
    
    try:
        for frame in range(start_frame, end_frame + 1):
            if not state.is_streaming:
                print("Streaming cancelled by user.")
                break
            
            # Send frame data directly without changing the current time in the UI
            if not send_frame_data(frame):
                print("Failed to send frame data. Stopping stream.")
                break

            frames_sent += 1
            if frames_sent % 100 == 0:
                print(f"Progress: Sent frame {frame}/{end_frame}")
                
    except Exception as e:
        print(f"An error occurred during sequential streaming: {e}")
        
    finally:
        # Restore original frame in Maya's timeline
        cmds.currentTime(original_frame, edit=True)
        
        total_time = time.time() - start_time
        if total_time > 0:
            print("\n--- Stream Complete ---")
            print(f"  Frames Sent: {frames_sent}/{end_frame - start_frame + 1}")
            print(f"  Total Time: {total_time:.2f}s")
            print(f"  Average FPS: {frames_sent / total_time:.1f}")
            print("-----------------------")
        
        # Reset state and UI in the main Maya thread
        cmds.evalDeferred(update_ui_after_stream_stop)

def update_ui_after_stream_stop():
    """Safely update the UI from the main thread after streaming stops."""
    state.is_streaming = False
    cmds.button("streamBtn", e=True, label="Stream Animation", bgc=(0.2, 0.5, 0.7))
    cmds.button("connectBtn", e=True, enable=True)
    cmds.inViewMessage(amg="Streaming finished.", pos="topCenter", fade=True)

# ============================================================================
# STREAMING CONTROL
# ============================================================================
def gather_all_frame_data():
    """
    RUNS IN THE MAIN THREAD. Gathers all animation data upfront.
    This will freeze Maya, which is the expected and safe behavior.
    """
    try:
        start_frame = int(cmds.playbackOptions(q=True, minTime=True))
        end_frame = int(cmds.playbackOptions(q=True, maxTime=True))
        print(f"Timeline range: {start_frame} to {end_frame}")
    except (TypeError, ValueError) as e:
        print(f"Error getting timeline: {e}")
        cmds.warning("Invalid animation range in timeline.")
        return None, 0

    if not state.actuator_mappings:
        print("No actuator mappings found")
        return None, 0

    print(f"Gathering data for frames {start_frame} to {end_frame}... Maya will be unresponsive.")
    print(f"Processing {len(state.actuator_mappings)} mappings:")
    for mapping in state.actuator_mappings:
        print(f"  - {mapping['maya_attr']} -> {mapping['mujoco_act']}")
    
    all_frames = []
    total_frames = end_frame - start_frame + 1

    # Start progress window
    cmds.progressWindow(
        title='Gathering Animation Data',
        progress=0,
        max=total_frames,
        status='Starting...',
        isInterruptable=True
    )

    try:
        for frame in range(start_frame, end_frame + 1):
            # Check if user cancelled
            if cmds.progressWindow(query=True, isCancelled=True):
                print("Data gathering cancelled by user")
                break

            # Update progress
            progress = frame - start_frame + 1
            cmds.progressWindow(
                edit=True,
                progress=progress,
                status=f'Processing Frame: {frame} ({progress}/{total_frames})'
            )

            frame_joints = {}
            mapping_success_count = 0
            
            for mapping in state.actuator_mappings:
                try:
                    maya_value = cmds.getAttr(mapping["maya_attr"], time=frame)
                    mujoco_value = map_value(
                        maya_value,
                        (mapping["maya_min"], mapping["maya_max"]),
                        (mapping["mujoco_min"], mapping["mujoco_max"])
                    )
                    frame_joints[mapping['mujoco_act']] = mujoco_value
                    mapping_success_count += 1
                except Exception as e:
                    print(f"Warning: Failed to read {mapping['maya_attr']} at frame {frame}: {e}")
                    continue
            
            if frame_joints:
                all_frames.append(frame_joints)
                
            # Print progress every 10 frames
            if (frame - start_frame) % 10 == 0:
                print(f"Frame {frame}: {mapping_success_count}/{len(state.actuator_mappings)} mappings successful")

    finally:
        # Always close progress window
        cmds.progressWindow(endProgress=1)

    print(f"Data gathering complete. Collected {len(all_frames)} frames with joint data.")
    
    if not all_frames:
        cmds.warning("No valid frame data collected. Check your mappings and timeline.")
        return None, 0
        
    return all_frames, start_frame

def stream_thread(all_frame_data, start_frame):
    """
    RUNS IN A BACKGROUND THREAD. Only handles sending data.
    Does NOT call any maya.cmds.
    """
    print(f"Background sending thread started for {len(all_frame_data)} frames.")
    frames_sent = 0
    start_time = time.time()
    
    try:
        for i, frame_joints in enumerate(all_frame_data):
            if not state.is_streaming:
                print("Streaming cancelled by user.")
                break
            
            frame_number = start_frame + i
            frame_package = {
                "type": "FRAME",
                "frame": frame_number,
                "timestamp": time.time(),
                "joints": frame_joints
            }

            try:
                message = (json.dumps(frame_package) + "\n").encode()
                state.socket_client.sendall(message)
                frames_sent += 1
                
                # Debug output for first few frames
                if frames_sent <= 5:
                    print(f"Sent frame {frame_number}: {len(frame_joints)} joints")
                    
            except socket.error as e:
                print(f"Connection error while sending frame {frame_number}: {e}")
                break # Stop if connection is lost
            except Exception as e:
                print(f"Unexpected error sending frame {frame_number}: {e}")
                break
            
            if frames_sent % 100 == 0:
                print(f"Progress: Sent frame {frame_number} ({frames_sent}/{len(all_frame_data)})")
                
            # Small delay to avoid overwhelming the network
            time.sleep(0.001)

    except Exception as e:
        print(f"An error occurred in the sending thread: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        total_time = time.time() - start_time
        print("\n--- Stream Complete ---")
        print(f"  Frames Sent: {frames_sent}/{len(all_frame_data)}")
        print(f"  Total Time: {total_time:.2f}s")
        if total_time > 0:
            print(f"  Average FPS: {frames_sent / total_time:.1f}")
        print("-----------------------")
        
        # Use evalDeferred to safely update UI from background thread
        cmds.evalDeferred(update_ui_after_stream_stop)

def toggle_stream(*args):
    """Starts or stops the animation stream."""
    if state.is_streaming:
        stop_stream()
    else:
        start_stream()

def start_stream():
    """Gathers data in the main thread, then starts the sending thread."""
    print("=== START STREAM CALLED ===")
    
    if not state.socket_client:
        cmds.warning("Not connected to MuJoCo server!")
        return
        
    if not state.actuator_mappings:
        cmds.warning("No joint mappings defined!")
        return

    # Validate mappings before starting
    print("Validating mappings...")
    invalid_mappings = []
    for mapping in state.actuator_mappings:
        if not cmds.objExists(mapping["maya_attr"].split('.')[0]):
            invalid_mappings.append(mapping["maya_attr"])
    
    if invalid_mappings:
        cmds.warning(f"Invalid Maya objects: {', '.join(invalid_mappings)}")
        return
        
    # STEP 1: Gather all data in the main thread (this will freeze the UI).
    print("Starting data gathering...")
    all_data, start_frame = gather_all_frame_data()
    
    if all_data is None or len(all_data) == 0:
        print("No data collected, aborting stream.")
        return

    print(f"Data collection complete. Starting network transmission...")
    state.is_streaming = True
    
    # STEP 2: Start the background thread to ONLY send the gathered data.
    state.stream_thread = threading.Thread(target=stream_thread, args=(all_data, start_frame))
    state.stream_thread.daemon = True
    state.stream_thread.start()
    
    print(f"Streaming thread started with thread ID: {state.stream_thread.ident}")
    
    # Update UI
    cmds.button("streamBtn", e=True, label="Stop Streaming", bgc=(0.8, 0.3, 0.3))
    cmds.button("connectBtn", e=True, enable=False)
    cmds.inViewMessage(amg="Streaming animation...", pos="topCenter", fade=True)

def stop_stream():
    """Signal the streaming thread to stop."""
    if state.is_streaming:
        print("Stopping stream...")
        state.is_streaming = False
        # The thread will see this flag and exit gracefully

# ============================================================================
# CONNECTION MANAGEMENT
# ============================================================================
def connect_mujoco(*args):
    """Establish connection to the MuJoCo server."""
    if state.socket_client:
        cmds.warning("Already connected!")
        return
    
    try:
        state.socket_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        state.socket_client.connect((Config.HOST, Config.PORT))
        state.socket_client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        state.socket_client.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, Config.SOCKET_BUFFER_SIZE)
        
        cmds.button("connectBtn", e=True, label="Disconnect", command=disconnect_mujoco, bgc=(0.6, 0.2, 0.2))
        cmds.button("streamBtn", e=True, enable=True)
        
        cmds.inViewMessage(amg=f"Connected to MuJoCo at {Config.HOST}:{Config.PORT}", pos="topCenter", fade=True)
        print(f"Successfully connected to MuJoCo at {Config.HOST}:{Config.PORT}")
        
    except Exception as e:
        state.socket_client = None
        cmds.warning(f"Connection failed: {e}")

def disconnect_mujoco(*args):
    """Disconnect from the MuJoCo server."""
    stop_stream()
    
    if state.socket_client:
        try:
            state.socket_client.close()
        except Exception as e:
            print(f"Error while closing socket: {e}")
    
    state.reset_connection()
    
    cmds.button("connectBtn", e=True, label="Connect", command=connect_mujoco, bgc=(0.3, 0.5, 0.8))
    cmds.button("streamBtn", e=True, enable=False)
    
    cmds.inViewMessage(amg="Disconnected from MuJoCo.", pos="topCenter", fade=True)
    print("Disconnected from MuJoCo.")

# ============================================================================
# MAPPING MANAGEMENT
# ============================================================================
def refresh_mapping_list():
    """Update the UI list of mappings."""
    cmds.textScrollList("mappingList", e=True, removeAll=True)
    
    for mapping in state.actuator_mappings:
        label = f"{mapping['maya_attr']} → {mapping['mujoco_act']}"
        cmds.textScrollList("mappingList", e=True, append=label)

def add_mapping(*args):
    """Add a new actuator mapping from UI fields."""
    maya_attr = cmds.textField("mayaAttr", q=True, text=True).strip()
    mujoco_act = cmds.textField("mujocoAct", q=True, text=True).strip()
    
    if not maya_attr or not mujoco_act:
        cmds.warning("Maya Attribute and MuJoCo Actuator fields are required.")
        return
    
    try:
        maya_min = float(cmds.floatField("mayaMin", q=True, value=True))
        maya_max = float(cmds.floatField("mayaMax", q=True, value=True))
        mujoco_min = float(cmds.floatField("mujocoMin", q=True, value=True))
        mujoco_max = float(cmds.floatField("mujocoMax", q=True, value=True))
    except ValueError:
        cmds.warning("Invalid range values. Please enter valid numbers.")
        return
    
    state.actuator_mappings.append({
        "maya_attr": maya_attr, "mujoco_act": mujoco_act,
        "maya_min": maya_min, "maya_max": maya_max,
        "mujoco_min": mujoco_min, "mujoco_max": mujoco_max
    })
    
    refresh_mapping_list()
    cmds.textField("mayaAttr", e=True, text="")
    cmds.textField("mujocoAct", e=True, text="")
    print(f"Added Mapping: {maya_attr} → {mujoco_act}")

def remove_mapping(*args):
    """Remove the selected mapping from the list."""
    selected_indices = cmds.textScrollList("mappingList", q=True, selectIndexedItem=True)
    if selected_indices:
        # Sort indices in reverse to avoid shifting issues when removing
        for idx in sorted(selected_indices, reverse=True):
            removed = state.actuator_mappings.pop(idx - 1)
            print(f"Removed Mapping: {removed['maya_attr']} → {removed['mujoco_act']}")
        refresh_mapping_list()

def save_mappings(*args):
    """Save the current mappings to a JSON file."""
    path = cmds.fileDialog2(fileMode=0, caption="Save Mappings", fileFilter="JSON (*.json)")
    if path and path[0]:
        with open(path[0], 'w') as f:
            json.dump(state.actuator_mappings, f, indent=4)
        cmds.inViewMessage(amg=f"Mappings saved to {path[0]}", pos="topCenter", fade=True)
        print(f"Mappings saved successfully.")

def load_mappings(*args):
    """Load mappings from a JSON file."""
    path = cmds.fileDialog2(fileMode=1, caption="Load Mappings", fileFilter="JSON (*.json)")
    if path and path[0]:
        try:
            with open(path[0], 'r') as f:
                state.actuator_mappings = json.load(f)
            refresh_mapping_list()
            cmds.inViewMessage(amg="Mappings loaded successfully.", pos="topCenter", fade=True)
            print("Mappings loaded.")
        except Exception as e:
            cmds.warning(f"Failed to load mappings: {e}")

# ============================================================================
# USER INTERFACE
# ============================================================================
def create_ui():
    """Create the main streamer UI window."""
    window_name = "mayaToMuJoCoStreamer"
    
    if cmds.window(window_name, exists=True):
        cmds.deleteUI(window_name)
    
    window = cmds.window(window_name, title="Maya-to-MuJoCo Streamer", widthHeight=(450, 600))
    
    main_layout = cmds.columnLayout(adjustableColumn=True, rowSpacing=10)

    # --- Mappings Section ---
    cmds.frameLayout(label="1. Joint Mappings", collapsable=True, marginHeight=5)
    cmds.columnLayout(adjustableColumn=True)
    
    cmds.text(label="Active Mappings:", align="left")
    cmds.textScrollList("mappingList", height=120, allowMultiSelection=True)
    cmds.popupMenu()
    cmds.menuItem(label="Remove Selected", command=remove_mapping)
    
    cmds.separator(style="in", height=15)
    
    cmds.text(label="Add New Mapping:", font="boldLabelFont", align="left")
    
    # Input fields for new mappings
    cmds.rowLayout(numberOfColumns=2, columnWidth2=(100, 325))
    cmds.text(label="Maya Attribute:")
    cmds.textField("mayaAttr", placeholderText="e.g., joint1.rotateX")
    cmds.setParent("..")
    
    cmds.rowLayout(numberOfColumns=2, columnWidth2=(100, 325))
    cmds.text(label="MuJoCo Actuator:")
    cmds.textField("mujocoAct", placeholderText="e.g., actuator_name")
    cmds.setParent("..")
    
    # Range fields
    cmds.rowLayout(numberOfColumns=5, columnWidth5=(80, 85, 30, 85, 100))
    cmds.text(label="Maya Range:")
    cmds.floatField("mayaMin", value=Config.DEFAULT_MAYA_RANGE[0], precision=1)
    cmds.text(label="to")
    cmds.floatField("mayaMax", value=Config.DEFAULT_MAYA_RANGE[1], precision=1)
    cmds.text(label=" (degrees)")
    cmds.setParent("..")
    
    cmds.rowLayout(numberOfColumns=5, columnWidth5=(80, 85, 30, 85, 100))
    cmds.text(label="MuJoCo Range:")
    cmds.floatField("mujocoMin", value=Config.DEFAULT_MUJOCO_RANGE[0], precision=2)
    cmds.text(label="to")
    cmds.floatField("mujocoMax", value=Config.DEFAULT_MUJOCO_RANGE[1], precision=2)
    cmds.text(label=" (radians)")
    cmds.setParent("..")
    
    cmds.separator(style="none", height=5)
    
    # Mapping action buttons
    cmds.rowLayout(numberOfColumns=4, columnAttach=[(i, 'both', 5) for i in range(1, 5)])
    cmds.button(label="Add Mapping", command=add_mapping, bgc=(0.3, 0.6, 0.3))
    cmds.button(label="Remove Selected", command=remove_mapping, bgc=(0.6, 0.3, 0.3))
    cmds.button(label="Save Mappings", command=save_mappings, bgc=(0.3, 0.3, 0.6))
    cmds.button(label="Load Mappings", command=load_mappings, bgc=(0.6, 0.6, 0.3))
    cmds.setParent("..") # end rowLayout
    cmds.setParent("..") # end columnLayout
    cmds.setParent("..") # end frameLayout
    
    # --- Control Section ---
    cmds.frameLayout(label="2. Connection & Streaming", collapsable=True, marginHeight=5)
    cmds.columnLayout(adjustableColumn=True, rowSpacing=7)
    
    cmds.button("connectBtn", label="Connect", command=connect_mujoco, 
                height=35, bgc=(0.3, 0.5, 0.8), 
                annotation="Connect to the MuJoCo server before streaming.")
                
    cmds.button("streamBtn", label="Stream Animation", command=toggle_stream,
                height=40, bgc=(0.2, 0.5, 0.7), enable=False,
                annotation="Stream the entire animation timeline to MuJoCo.")
                
    cmds.setParent("..") # end columnLayout
    cmds.setParent("..") # end frameLayout
    
    cmds.showWindow(window)
    refresh_mapping_list()

# ============================================================================
# SCRIPT EXECUTION
# ============================================================================
create_ui()
print("\n" + "=" * 50)
print("MuJoCo Animation Streamer Loaded")
print(f"Ready to connect to server at: {Config.HOST}:{Config.PORT}")
print("=" * 50)