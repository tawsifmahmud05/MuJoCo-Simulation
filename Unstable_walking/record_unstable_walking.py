#!/usr/bin/env python3
"""
Record Unstable Walking Video

This script records the unstable walking behavior and saves it as a video file.
"""

import numpy as np
import mujoco
import mediapy as media
import imageio
from pathlib import Path
from tqdm import tqdm


def main():
    # Model path
    model_dir = Path("Model/unitree_go1")
    model_xml = model_dir / "scene.xml"
    
    print(f"Loading model from: {model_xml}")
    
    # Load model
    model = mujoco.MjModel.from_xml_path(str(model_xml))
    data = mujoco.MjData(model)
    
    print(f"Model loaded successfully!")
    print(f"- Model name: Go1 Robot")
    print(f"- Number of bodies: {model.nbody}")
    print(f"- Number of joints: {model.njnt}")
    print(f"- Number of actuators: {model.nu}")
    print(f"- Number of keyframes: {model.nkey}")
    
    # Initialize to first keyframe if available
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)
        ctrl_baseline = data.ctrl.copy()
        print(f"Initialized to keyframe 0")
    else:
        mujoco.mj_resetData(model, data)
        ctrl_baseline = np.mean(model.actuator_ctrlrange, axis=1)
        print("No keyframes available, using default pose")
    
    # Video parameters
    duration = 15.0  # seconds of recording
    fps = 30  # frames per second
    width, height = 640, 480
    
    # Set up renderer
    model.vis.global_.offheight = height
    model.vis.global_.offwidth = width
    renderer = mujoco.Renderer(model, height=height, width=width)
    
    # Camera setup for good viewing angle
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    camera.distance = 2.5
    camera.azimuth = 45
    camera.elevation = -20
    
    # Walking parameters - EXACT MATCH to unstable_walking.py
    frequency = 0.5  # Hz - matching unstable_walking.py
    thigh_amplitude = 0.3   # Matching unstable_walking.py  
    calf_amplitude = 0.4    # Matching unstable_walking.py
    
    # Calculate simulation steps
    nsteps = int(np.ceil(duration / model.opt.timestep))
    frame_skip = max(1, int((1.0 / fps) / model.opt.timestep))  # How many sim steps per frame
    
    print(f"\nRecording parameters:")
    print(f"- Duration: {duration}s")
    print(f"- FPS: {fps}")
    print(f"- Resolution: {width}x{height}")
    print(f"- Total simulation steps: {nsteps}")
    print(f"- Frame skip: {frame_skip}")
    
    frames = []
    walking_started = False
    
    print(f"\n🎬 Starting recording...")
    
    # Simulation and recording loop
    for step in tqdm(range(nsteps), desc="Recording unstable walking"):
        time = data.time
        
        # Control logic (same as unstable_walking.py)
        if time > 2:  # Start walking after 2 seconds
            if not walking_started:
                walking_started = True
            
            # Create walking phases
            left_legs_phase = 2 * np.pi * frequency * time
            right_legs_phase = left_legs_phase + np.pi
            
            # EXACT MATCH to unstable_walking.py - no ramp up, direct amplitudes
            # Use the exact same leg lifting logic as original
            fl_lift = np.max([0, np.sin(left_legs_phase)])  # Only positive = lift
            rl_lift = np.max([0, np.sin(left_legs_phase)])  # Only positive = lift
            fr_lift = np.max([0, np.sin(right_legs_phase)])  # Only positive = lift
            rr_lift = np.max([0, np.sin(right_legs_phase)])  # Only positive = lift
            
            # Apply walking control - EXACT MATCH to unstable_walking.py
            # LEFT LEGS (FL + RL)
            data.ctrl[4] = ctrl_baseline[4] + thigh_amplitude * np.sin(left_legs_phase)   # FL_thigh
            data.ctrl[5] = ctrl_baseline[5] + calf_amplitude * fl_lift                    # FL_calf
            data.ctrl[10] = ctrl_baseline[10] + thigh_amplitude * np.sin(left_legs_phase) # RL_thigh
            data.ctrl[11] = ctrl_baseline[11] + calf_amplitude * rl_lift                  # RL_calf
            
            # RIGHT LEGS (FR + RR)
            data.ctrl[1] = ctrl_baseline[1] + thigh_amplitude * np.sin(right_legs_phase)  # FR_thigh
            data.ctrl[2] = ctrl_baseline[2] + calf_amplitude * fr_lift                    # FR_calf
            data.ctrl[7] = ctrl_baseline[7] + thigh_amplitude * np.sin(right_legs_phase)  # RR_thigh
            data.ctrl[8] = ctrl_baseline[8] + calf_amplitude * rr_lift                    # RR_calf
            
            # Keep hip joints stable
            for i in [0, 3, 6, 9]:
                data.ctrl[i] = ctrl_baseline[i]
        else:
            # Stabilization phase
            for i in range(model.nu):
                data.ctrl[i] = ctrl_baseline[i]
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        # Capture frame at desired fps
        if step % frame_skip == 0:
            # Update camera to slowly rotate around robot
            camera.azimuth = 45 + 60 * np.sin(0.1 * time)  # Slow rotation
            
            # Render frame
            renderer.update_scene(data, camera)
            frame = renderer.render()
            frames.append(frame.copy())
    
    print(f"\n✅ Recording complete! Captured {len(frames)} frames")
    
    # Save video
    output_path = "media/unstable_walking.mp4"
    print(f"💾 Saving video to: {output_path}")
    
    # Create media directory if it doesn't exist
    Path("media").mkdir(exist_ok=True)
    
    # Save as MP4
    imageio.mimsave(output_path, frames, fps=fps)
    
    # Also create a GIF for web viewing
    gif_path = "media/unstable_walking.gif"
    print(f"🎞️ Creating GIF: {gif_path}")
    
    # Reduce frame rate for GIF and skip some frames to reduce size
    gif_frames = frames[::2]  # Take every 2nd frame
    imageio.mimsave(gif_path, gif_frames, fps=fps//2)
    
    print(f"\n🎉 Video recording complete!")
    print(f"📹 MP4: {output_path}")
    print(f"🎞️ GIF: {gif_path}")
    print(f"📊 Video stats:")
    print(f"   - Duration: {len(frames)/fps:.1f} seconds")
    print(f"   - Frames: {len(frames)}")
    print(f"   - Resolution: {width}x{height}")


if __name__ == "__main__":
    main()
