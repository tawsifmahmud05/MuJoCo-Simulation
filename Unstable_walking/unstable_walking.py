import numpy as np
import mujoco
import mujoco.viewer
from pathlib import Path


def main():
    # Model path - you can change this to any model in the menagerie
    model_dir = Path("Model/unitree_go1")
    model_xml = model_dir / "scene.xml"
    
    print(f"Loading model from: {model_xml}")
    
    # Load model
    model = mujoco.MjModel.from_xml_path(str(model_xml))
    data = mujoco.MjData(model)
    
    print(f"Model loaded successfully!")
    
    # Extract model name safely
    null_byte = b'\x00'
    model_name = model.names[1:model.names.find(null_byte, 1)].decode()
    print(f"- Model name: {model_name}")
    print(f"- Number of bodies: {model.nbody}")
    print(f"- Number of joints: {model.njnt}")
    print(f"- Number of actuators: {model.nu}")
    print(f"- Number of keyframes: {model.nkey}")
    
    # Initialize to first keyframe if available and get stable control baseline
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)  # Forward step to get proper control values
        ctrl_baseline = data.ctrl.copy()
        print(f"Initialized to keyframe 0")
    else:
        mujoco.mj_resetData(model, data)
        # Use middle of actuator control range as baseline
        ctrl_baseline = np.mean(model.actuator_ctrlrange, axis=1)
        print("No keyframes available, using default pose")
    
    # Initializing robot in a stable pose (keeping feet flat on the ground)
    for i in range(model.nu):
        data.ctrl[i] = ctrl_baseline[i]

    # Launch the viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Set initial camera position
        viewer.cam.distance = 2.0
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -15
        
        print("\nViewer Controls:")
        print("- Mouse: Rotate view (left click + drag)")
        print("- Mouse wheel: Zoom in/out")
        print("- Right click + drag: Pan")
        print("- Space: Pause/unpause simulation")
        print("- Backspace: Reset simulation")
        print("- Tab: Cycle through cameras")
        print("- '[' / ']': Previous/next keyframe")
        print("- 'V': Toggle visualization options")
        print("- 'C': Toggle contact visualization")
        print("- 'T': Toggle transparency")
        print("- 'F': Toggle frame visualization")
        print("- 'H': Toggle help overlay")
        print("- Esc or close window: Exit")
        
        # Walking pattern parameters - very conservative for stability
        frequency = 0.5  # Hz (much slower for stability)
        amplitude = 0.15  # Very small amplitude to prevent instability
        walking_started = False

        # Simulation loop
        while viewer.is_running():
            time = data.time  # Get current time in simulation
            
            # Step the simulation
            mujoco.mj_step(model, data)
            
            # Start walking after 2 seconds of stabilizing the robot
            if time > 2:
                if not walking_started:
                    print(f"🐕 Starting walking at time: {time:.1f}s")
                    walking_started = True
                
                # Actuator order from XML: FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf, 
                #                         RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf
                # Indices:                   0,      1,       2,     3,      4,       5,
                #                           6,      7,       8,     9,     10,      11
                
                # Try a simpler forward walking pattern
                # Create alternating leg motion: left legs vs right legs
                left_phase = 2 * np.pi * frequency * time           # FL + RL together
                right_phase = left_phase + np.pi                    # FR + RR together (opposite)
                
                # Focus on thigh joints with coordinated calf movement for propulsion
                # Keyframe shows: ctrl="0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8"
                # This suggests: hip=0, thigh=0.9, calf=-1.8 for standing pose
                
                # PROPER WALKING with leg lifting - coordinate thigh and calf
                # Create walking phases for leg lifting and placement
                left_legs_phase = 2 * np.pi * frequency * time        # FL + RL together
                right_legs_phase = left_legs_phase + np.pi            # FR + RR together (opposite)
                
                # Walking parameters
                thigh_amplitude = 0.3   # Thigh movement for forward/backward
                calf_amplitude = 0.4    # Calf movement for leg lifting
                
                # LEFT LEGS (FL + RL) - lift when sine is positive
                fl_lift = np.max([0, np.sin(left_legs_phase)])  # Only positive = lift
                rl_lift = np.max([0, np.sin(left_legs_phase)])  # Only positive = lift
                
                # FL (indices 3,4,5)
                data.ctrl[4] = ctrl_baseline[4] + thigh_amplitude * np.sin(left_legs_phase)   # FL_thigh: forward/back
                data.ctrl[5] = ctrl_baseline[5] + calf_amplitude * fl_lift                    # FL_calf: lift leg
                
                # RL (indices 9,10,11)  
                data.ctrl[10] = ctrl_baseline[10] + thigh_amplitude * np.sin(left_legs_phase) # RL_thigh: forward/back
                data.ctrl[11] = ctrl_baseline[11] + calf_amplitude * rl_lift                  # RL_calf: lift leg
                
                # RIGHT LEGS (FR + RR) - lift when sine is positive
                fr_lift = np.max([0, np.sin(right_legs_phase)])  # Only positive = lift
                rr_lift = np.max([0, np.sin(right_legs_phase)])  # Only positive = lift
                
                # FR (indices 0,1,2)
                data.ctrl[1] = ctrl_baseline[1] + thigh_amplitude * np.sin(right_legs_phase)  # FR_thigh: forward/back
                data.ctrl[2] = ctrl_baseline[2] + calf_amplitude * fr_lift                    # FR_calf: lift leg
                
                # RR (indices 6,7,8)
                data.ctrl[7] = ctrl_baseline[7] + thigh_amplitude * np.sin(right_legs_phase)  # RR_thigh: forward/back  
                data.ctrl[8] = ctrl_baseline[8] + calf_amplitude * rr_lift                    # RR_calf: lift leg
                
                # Keep hip joints stable for balance
                for i in [0, 3, 6, 9]:  # hip joints
                    data.ctrl[i] = ctrl_baseline[i]
                    
                # Progress indicator
                if int(time) % 2 == 0 and abs(time - int(time)) < 0.02:
                    print(f"🚶 Walking... {time:.1f}s")
            else:
                # Stabilization phase - keep all joints at baseline
                for i in range(model.nu):
                    data.ctrl[i] = ctrl_baseline[i]

            # Sync the viewer
            viewer.sync()
        
        print("Viewer closed. Goodbye!")

if __name__ == "__main__":
    main()
