#!/usr/bin/env python3
"""
MuJoCo Viewer Demo Script

This script demonstrates the same simulation as the tutorial notebook
but uses the MuJoCo viewer for real-time visualization and interaction.
"""

import numpy as np
import mujoco
import mujoco.viewer
from pathlib import Path
import time


def quartic(t: float) -> float:
    """Quartic smoothing function."""
    return 0 if abs(t) > 1 else (1 - t**2) ** 2


def blend_coef(t: float, duration: float, std: float) -> float:
    """Blend coefficient for smooth transitions."""
    normalised_time = 2 * t / duration - 1
    return quartic(normalised_time / std)


def unit_smooth(normalised_time: float) -> float:
    """Unit smoothing function using cosine."""
    return 1 - np.cos(normalised_time * 2 * np.pi)


def azimuth(time: float, duration: float, total_rotation: float, offset: float) -> float:
    """Calculate camera azimuth for smooth rotation."""
    return offset + unit_smooth(time / duration) * total_rotation


def main():
    # Parameters
    duration = 10.0  # seconds
    ctrl_rate = 2
    ctrl_std = 0.05
    total_rot = 60  # degrees
    blend_std = 0.8
    
    # Model path
    model_dir = Path("unitree_go1")
    model_xml = model_dir / "scene.xml"
    
    print(f"Loading model from: {model_xml}")
    
    # Load model
    model = mujoco.MjModel.from_xml_path(str(model_xml))
    data = mujoco.MjData(model)
    
    # Set random seed for reproducibility
    np.random.seed(12345)
    
    # Sample actuator noise and smooth it
    nsteps = int(np.ceil(duration / model.opt.timestep))
    perturb = np.random.randn(nsteps, model.nu)
    width = int(nsteps * ctrl_rate / duration)
    kernel = np.exp(-0.5 * np.linspace(-3, 3, width) ** 2)
    kernel /= np.linalg.norm(kernel)
    
    for i in range(model.nu):
        perturb[:, i] = np.convolve(perturb[:, i], kernel, mode="same")
    
    # Set the desired control point
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)  # Ensure proper initialization
        ctrl0 = data.ctrl.copy()
    else:
        mujoco.mj_resetData(model, data)
        ctrl0 = np.mean(model.actuator_ctrlrange, axis=1)
    
    print(f"Model loaded successfully!")
    print(f"- Timestep: {model.opt.timestep:.6f} s")
    print(f"- Total steps: {nsteps}")
    print(f"- Number of actuators: {model.nu}")
    print(f"- Number of keyframes: {model.nkey}")
    
    # Launch the viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Set initial camera position
        viewer.cam.distance = 1.5
        viewer.cam.azimuth = model.vis.global_.azimuth
        viewer.cam.elevation = -20
        
        # Enable visualization options
        viewer.opt.geomgroup[2] = True  # Visual geoms
        viewer.opt.geomgroup[3] = False  # Collision geoms initially off
        
        print("\nStarting simulation...")
        print("Press 'V' to toggle between visual and collision geometries")
        print("Press 'Space' to pause/unpause")
        print("Press 'R' to reset")
        print("Press 'Esc' or close window to exit")
        
        start_time = time.time()
        step = 0
        
        # Main simulation loop
        while viewer.is_running() and step < nsteps:
            # Apply control with noise
            data.ctrl[:] = ctrl0 + ctrl_std * perturb[step]
            
            # Step the simulation
            mujoco.mj_step(model, data)
            
            # Update camera for smooth rotation
            elapsed_time = data.time
            if elapsed_time <= duration:
                viewer.cam.azimuth = azimuth(
                    elapsed_time, duration, total_rot, model.vis.global_.azimuth
                )
            
            # Blend between visual and collision geometries based on time
            blend_factor = blend_coef(elapsed_time, duration, blend_std)
            if blend_factor > 0.5:
                # Show collision geometries when blend factor is high
                viewer.opt.geomgroup[2] = False  # Visual geoms off
                viewer.opt.geomgroup[3] = True   # Collision geoms on
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL] = True
            else:
                # Show visual geometries when blend factor is low
                viewer.opt.geomgroup[2] = True   # Visual geoms on
                viewer.opt.geomgroup[3] = False  # Collision geoms off
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL] = False
            
            # Sync with real time (approximately)
            viewer.sync()
            
            step += 1
            
            # Print progress occasionally
            if step % 500 == 0:
                progress = (step / nsteps) * 100
                print(f"Progress: {progress:.1f}% (Time: {elapsed_time:.2f}s)")
        
        print(f"\nSimulation completed!")
        print(f"Total simulation time: {data.time:.2f} seconds")
        print(f"Real time elapsed: {time.time() - start_time:.2f} seconds")
        
        # Keep viewer open for inspection
        print("\nViewer will remain open for inspection...")
        print("Close the viewer window to exit.")
        while viewer.is_running():
            viewer.sync()


if __name__ == "__main__":
    main()
