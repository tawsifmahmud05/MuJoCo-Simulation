#!/usr/bin/env python3
"""
Interactive MuJoCo Viewer

A simple interactive viewer for MuJoCo models with keyboard controls.
"""

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
        
        current_keyframe = 0
        
        # Simulation loop
        while viewer.is_running():
            # Add small control noise around stable baseline for interesting motion
            if model.nu > 0:
                noise = 0.02 * np.random.randn(model.nu)  # Much smaller noise
                data.ctrl[:] = ctrl_baseline + noise
            
            # Step the simulation
            mujoco.mj_step(model, data)
            
            # Sync the viewer
            viewer.sync()
            
            # Handle keyframe switching with bracket keys
            # Note: This is a simple implementation - MuJoCo viewer handles most keys automatically
            
        print("Viewer closed. Goodbye!")


if __name__ == "__main__":
    main()
