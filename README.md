# MuJoCo Project - Unitree Go1 Robot Simulation

This project demonstrates MuJoCo physics simulation using the Unitree Go1 quadruped robot model. It includes both interactive viewer demos and Jupyter notebook tutorials for robotics simulation and visualization.

## 📸 Preview

### Screenshot

![Project Screenshot](media/screenshot.png)

### Demo Animations

**Stable Simulation:**
![Stable Simulation Demo](media/demo.gif)

**Unstable Walking Research:**
![Unstable Walking Demo](media/unstable_walking.gif)

_The demo animations show the Unitree Go1 robot simulation with different control algorithms. The first shows stable simulation with camera rotation, while the second demonstrates unstable walking gait research._

> **Note:** Full videos available at [`media/output.mp4`](media/output.mp4) and [`media/unstable_walking.mp4`](media/unstable_walking.mp4)

## 🚀 Project Overview

This repository contains:

- **MuJoCo physics simulation** of the Unitree Go1 robot
- **Interactive viewer demos** with real-time visualization
- **Jupyter notebook tutorials** for learning MuJoCo
- **Walking gait research** including unstable walking experiments
- **Video rendering capabilities** for creating simulation outputs

## 📁 Project Structure

```
MuJoCo Project/
├── tutorial.ipynb              # Jupyter notebook with MuJoCo tutorial
├── mujoco_viewer_demo.py       # Interactive viewer demonstration
├── interactive_viewer.py       # Additional interactive viewer script
├── requirements.txt            # Python dependencies
├── .gitignore                 # Git ignore rules
├── README.md                  # Project documentation
├── MJDATA.TXT                 # MuJoCo debug/memory data
├── media/                     # 📁 Media files
│   ├── demo.gif               # 🎬 Stable simulation demo (for README)
│   ├── unstable_walking.gif   # 🎬 Unstable walking demo (for README)
│   ├── output.mp4             # 🎥 Full stable simulation video
│   ├── unstable_walking.mp4   # 🎥 Full unstable walking video
│   └── screenshot.png         # 📸 Project screenshot
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
   git clone <your-repo-url>
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

### 1. Interactive Viewer Demo

Run the real-time interactive simulation:

```bash
python mujoco_viewer_demo.py
```

**Features:**

- Real-time physics simulation
- Interactive camera controls
- Smooth camera rotation around the robot
- Toggle between visual and collision geometries
- Keyboard controls:
  - `Space` - Pause/unpause simulation
  - `R` - Reset simulation
  - `V` - Toggle geometry display
  - `Esc` - Exit

### 2. Additional Interactive Viewer

```bash
python interactive_viewer.py
```

### 3. Jupyter Notebook Tutorial

Launch Jupyter and open the tutorial:

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

### 4. Unstable Walking Research

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

## 🎥 Video Generation

The tutorial notebook demonstrates how to:

1. Set up cameras with smooth rotation
2. Apply control noise for realistic motion
3. Blend between visual and collision geometries
4. Render high-quality videos
5. Export as MP4 files

**Example outputs:**

- **Stable Simulation GIF:** [`media/demo.gif`](media/demo.gif) - Smooth simulation with camera rotation
- **Unstable Walking GIF:** [`media/unstable_walking.gif`](media/unstable_walking.gif) - Research locomotion patterns
- **Stable Video:** [`media/output.mp4`](media/output.mp4) - High-quality 10-second simulation with dynamic motion
- **Unstable Walking Video:** [`media/unstable_walking.mp4`](media/unstable_walking.mp4) - 15-second unstable walking research footage

## 🤖 Robot Model: Unitree Go1

The project uses the Unitree Go1 quadruped robot model featuring:

- **12 degrees of freedom** (3 per leg)
- **Realistic joint limits** and dynamics
- **High-quality mesh models** for visual rendering
- **Collision geometries** for physics simulation
- **Multiple keyframes** for different poses

## 🔧 Customization

### Simulation Parameters

Key parameters you can modify:

```python
# In mujoco_viewer_demo.py or tutorial.ipynb
duration = 10.0          # Simulation duration (seconds)
ctrl_rate = 2            # Control update rate
ctrl_std = 0.05          # Control noise standard deviation
total_rot = 60           # Camera rotation (degrees)
fps = 60                 # Video frame rate
```

### Camera Settings

```python
camera.distance = 1.5    # Distance from robot
camera.azimuth = 0       # Horizontal angle
camera.elevation = -20   # Vertical angle
```

### Rendering Options

```python
# Resolution options
Resolution.SD = (480, 640)     # Standard definition
Resolution.HD = (720, 1280)    # High definition
Resolution.UHD = (2160, 3840)  # Ultra high definition
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
