# MuJoCo Project - Robotics Simulation & Learning

This project demonstrates MuJoCo physics simulation with educational robotics modules and research applications. It features the Unitree Go1 quadruped robot model, forward kinematics learning modules, and unstable walking gait research.

## 📸 Preview

### Screenshot

![Project Screenshot](media/screenshot.png)

### Demo Animations

**Unstable Walking Research:**
![Unstable Walking Demo](media/unstable_walking.gif)

_The demo animation shows the Unitree Go1 robot simulation demonstrating unstable walking gait research._

> **Note:** Full video available at [`media/unstable_walking.mp4`](media/unstable_walking.mp4)

## 🚀 Project Overview

This repository contains:

- **Educational robotics modules** with forward kinematics learning
- **MuJoCo physics simulation** of the Unitree Go1 quadruped robot
- **Interactive 3D visualization** and real-time simulation
- **Jupyter notebook tutorials** for learning MuJoCo fundamentals
- **Research applications** including unstable walking gait experiments
- **Video rendering capabilities** for creating simulation outputs
- **Customizable templates** for building your own simulations

## 📁 Project Structure

```
MuJoCo Project/
├── tutorial.ipynb              # Jupyter notebook with MuJoCo tutorial
├── mujoco_viewer_demo.py       # Interactive viewer demonstration
├── interactive_viewer.py       # Additional interactive viewer script
├── requirements.txt            # Python dependencies
├── README.md                  # Project documentation
├── MJDATA.TXT                 # MuJoCo debug/memory data
├── media/                     # 📁 Media files
│   ├── demo.gif               # 🎬 Demo simulation
│   ├── output.mp4             # 🎥 Demo video output
│   ├── unstable_walking.gif   # 🎬 Unstable walking demo
│   ├── unstable_walking.mp4   # 🎥 Unstable walking video
│   └── screenshot.png         # 📸 Project screenshot
├── Learning/                  # 📚 Educational robotics modules
│   ├── 1_Planar Manipulator/  # Basic planar manipulator setup
│   │   └── planar_3R.xml      # 3R planar manipulator model
│   └── 2_Planar3R_Forward_Kinematics/ # Forward kinematics learning
│       ├── mj_planar3R_forward_kinematics.py # MuJoCo forward kinematics demo
│       ├── forward_kinematics.py # Python forward kinematics implementation
│       └── planar_3R.xml      # 3R manipulator model
├── Unstable_walking/          # 🔬 Walking gait research
│   ├── unstable_walking.py    # Interactive unstable walking demo
│   └── record_unstable_walking.py # Video recording script
├── Model/                     # 🤖 Robot models
│   └── unitree_go1/           # Unitree Go1 robot model
│       ├── go1.xml            # Main robot model file
│       ├── scene.xml          # Complete scene with robot
│       ├── go1.png            # Robot image
│       ├── assets/            # Robot mesh files
│       │   ├── calf.stl
│       │   ├── hip.stl
│       │   ├── thigh.stl
│       │   ├── thigh_mirror.stl
│       │   └── trunk.stl
│       ├── CHANGELOG.md
│       ├── LICENSE
│       └── README.md
└── venv/                      # Python virtual environment
```

## 🛠️ Setup Instructions

### Prerequisites

- Python 3.8 or higher
- Git (optional, for cloning)

### Installation

1. **Clone or download the project**

   ```bash
   git clone https://github.com/tawsifmahmud05/MuJoCo-Simulation
   cd "MuJoCo Project"
   ```

2. **Create and activate virtual environment**

   ```bash
   # Create virtual environment
   python -m venv venv

   # Activate virtual environment
   # On Windows:
   .\venv\Scripts\activate

   # On macOS/Linux:
   source venv/bin/activate
   ```

3. **Install dependencies**

   ```bash
   pip install --upgrade pip
   pip install -r requirements.txt
   ```

4. **Verify installation**
   ```bash
   python -c "import mujoco, numpy, mediapy; print('Installation successful!')"
   ```

## 📦 Dependencies

The project uses the following main packages:

- **mujoco (≥3.0.0)** - Physics simulation engine
- **numpy (≥1.21.0)** - Numerical computations
- **mediapy (≥1.1.0)** - Media processing and visualization
- **tqdm (≥4.64.0)** - Progress bars
- **imageio (≥2.19.0)** - Video/image I/O
- **pathlib2 (≥2.3.0)** - Path handling utilities

See `requirements.txt` for the complete list with all dependencies.

## 🎮 Usage

### 1. Learning Modules - Robotics Education

Start with the educational modules to learn robotics fundamentals:

#### 1.1 Planar 3R Manipulator Forward Kinematics

Learn forward kinematics with a 3-revolute joint planar manipulator:

```bash
# Run the MuJoCo forward kinematics demonstration
cd Learning/2_Planar3R_Forward_Kinematics
python mj_planar3R_forward_kinematics.py
```

**Learning Features:**

- **Interactive 3D visualization** of 3R planar manipulator
- **Forward kinematics comparison** between MuJoCo and Python implementations
- **Real-time position tracking** of end-effector
- **Educational template** (`template_mujoco.py`) for custom simulations
- **Mathematical verification** of forward kinematics calculations

**Key Learning Outcomes:**

- Understanding of homogeneous transformation matrices
- Comparison of physics engine vs. analytical solutions
- Interactive manipulation of joint angles
- Visualization of workspace and reachable space

#### 1.2 Custom Simulation Development

Create your own MuJoCo simulations using the existing code as reference:

```bash
# Use the forward kinematics demo as a starting point
cp Learning/2_Planar3R_Forward_Kinematics/mj_planar3R_forward_kinematics.py my_simulation.py
```

**Development Features:**

- Complete MuJoCo setup with OpenGL rendering
- Interactive camera controls (mouse and keyboard)
- Controller framework for custom algorithms
- Configurable simulation parameters
- Forward kinematics implementation examples

### 2. Jupyter Notebook Tutorial

Launch Jupyter and open the comprehensive tutorial:

```bash
jupyter notebook tutorial.ipynb
```

**Tutorial includes:**

- MuJoCo basics and model loading
- Rendering and visualization techniques
- Camera control and animation
- Video generation and export
- Keyframe animation
- Physics parameter exploration

### 3. Unstable Walking Research

Explore quadruped locomotion algorithms:

```bash
# Interactive unstable walking demo
python Unstable_walking/unstable_walking.py

# Record video of unstable walking
python Unstable_walking/record_unstable_walking.py
```

**Research Features:**

- Real-time unstable walking simulation
- Leg lifting and ground contact dynamics
- Experimental gait patterns
- Video recording for analysis
- Parameter tuning for different behaviors

## 🎥 Video Generation & Media

The tutorial notebook demonstrates how to:

1. Set up cameras with smooth rotation
2. Apply control noise for realistic motion
3. Blend between visual and collision geometries
4. Render high-quality videos
5. Export as MP4 files

**Available Media:**

- **Demo GIF:** [`media/demo.gif`](media/demo.gif) - General simulation demonstration
- **Demo Video:** [`media/output.mp4`](media/output.mp4) - Demo video output
- **Unstable Walking GIF:** [`media/unstable_walking.gif`](media/unstable_walking.gif) - Research locomotion patterns
- **Unstable Walking Video:** [`media/unstable_walking.mp4`](media/unstable_walking.mp4) - 15-second unstable walking research footage
- **Project Screenshot:** [`media/screenshot.png`](media/screenshot.png) - Overview of the simulation environment

## 🤖 Robot Models

### Unitree Go1 Quadruped Robot

The project uses the Unitree Go1 quadruped robot model featuring:

- **12 degrees of freedom** (3 per leg)
- **Realistic joint limits** and dynamics
- **High-quality mesh models** for visual rendering
- **Collision geometries** for physics simulation
- **Multiple keyframes** for different poses

### Planar 3R Manipulator

The learning modules include a 3-revolute joint planar manipulator:

- **3 degrees of freedom** in the XY plane
- **Link lengths**: l1=1.0m, l2=1.0m, l3=0.25m
- **Forward kinematics** using homogeneous transformation matrices
- **Interactive visualization** with real-time position tracking
- **Educational comparison** between MuJoCo and analytical solutions

## 🔧 Customization & Development

### Simulation Parameters

Key parameters you can modify in your simulations:

```python
# In tutorial.ipynb or custom simulations
duration = 10.0          # Simulation duration (seconds)
ctrl_rate = 2            # Control update rate
ctrl_std = 0.05          # Control noise standard deviation
total_rot = 60           # Camera rotation (degrees)
fps = 60                 # Video frame rate
```

### Camera Settings

```python
# Camera configuration for optimal viewing
camera.distance = 1.5    # Distance from robot
camera.azimuth = 0       # Horizontal angle
camera.elevation = -20   # Vertical angle
camera.lookat = [0, 0, 0] # Focus point
```

### Rendering Options

```python
# Resolution options for video generation
Resolution.SD = (480, 640)     # Standard definition
Resolution.HD = (720, 1280)    # High definition
Resolution.UHD = (2160, 3840)  # Ultra high definition
```

### Creating Custom Simulations

Use the existing code as a starting point for your own simulations:

```python
# Copy the forward kinematics demo and modify
cp Learning/2_Planar3R_Forward_Kinematics/mj_planar3R_forward_kinematics.py my_simulation.py

# Key areas to customize:
# 1. XML model path
# 2. Controller logic in controller() function
# 3. Initialization in init_controller() function
# 4. Camera settings for visualization
# 5. Forward kinematics calculations
```

## 🐛 Troubleshooting

### Common Issues

1. **Import Error: No module named 'mujoco'**

   - Ensure virtual environment is activated
   - Reinstall with: `pip install mujoco>=3.0.0`

2. **OpenGL/Graphics Issues**

   - Update graphics drivers
   - Try software rendering: `export MUJOCO_GL=osmesa`

3. **Memory Issues**

   - Reduce video resolution
   - Decrease simulation duration
   - Close other applications

4. **Model Loading Errors**
   - Check that `unitree_go1/scene.xml` exists
   - Verify all mesh files are in `unitree_go1/assets/`

## 📊 Performance Notes

- **Simulation speed**: ~28 steps/second on typical hardware
- **Memory usage**: ~500MB for standard simulations
- **Video rendering**: Real-time for SD, slower for HD/UHD
- **Learning modules**: Optimized for educational use with clear visualizations
- **Research simulations**: Configurable for high-performance computing

## 🎓 Educational Value

This project is designed for:

- **Students** learning robotics and simulation
- **Researchers** exploring locomotion algorithms
- **Developers** building custom MuJoCo applications
- **Educators** teaching robotics concepts

The structured learning modules provide a clear progression from basic forward kinematics to advanced simulation techniques.
