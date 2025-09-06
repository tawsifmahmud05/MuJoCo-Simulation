# MuJoCo Robotics Learning Project

This project provides a comprehensive educational journey through robotics simulation using MuJoCo, featuring progressive learning modules from basic planar manipulators to advanced UR5 robot kinematics and inverse kinematics.

## 📸 Preview

### Demo Animations

**Unstable Walking Research:**
![Unstable Walking Demo](media/unstable_walking.gif)

_The demo animation shows the Unitree Go1 robot simulation demonstrating unstable walking gait research._

> **Note:** Full video available at [`media/unstable_walking.mp4`](media/unstable_walking.mp4)

**General Demo:**
![Demo Simulation](media/demo.gif)

_General simulation demonstration showcasing MuJoCo physics capabilities._

## 🎓 Learning Modules Overview

This repository contains a structured learning path through robotics simulation, organized into progressive modules:

### 1. Planar Manipulator (Module 1)

**Location:** `Learning/1_Planar Manipulator/`

- **Purpose:** Introduction to basic robotic manipulator concepts
- **Model:** 3R planar manipulator (`planar_3R.xml`)
- **Learning Focus:** Understanding joint configurations and basic robot structure
- **Features:**
  - Simple 3-revolute joint configuration
  - Planar motion constraints
  - Foundation for more complex manipulators

### 2. Planar 3R Forward Kinematics (Module 2)

**Location:** `Learning/2_Planar3R_Forward_Kinematics/`

- **Purpose:** Learn forward kinematics calculations and verification
- **Key Files:**
  - `mj_planar3R_forward_kinematics.py` - MuJoCo simulation with interactive visualization
  - `forward_kinematics.py` - Python implementation of forward kinematics
  - `planar_3R.xml` - 3R manipulator model

**Learning Features:**

- **Interactive 3D visualization** of 3R planar manipulator
- **Forward kinematics comparison** between MuJoCo and Python implementations
- **Real-time position tracking** of end-effector
- **Mathematical verification** of forward kinematics calculations
- **Educational template** for custom simulations

**Key Learning Outcomes:**

- Understanding of homogeneous transformation matrices
- Comparison of physics engine vs. analytical solutions
- Interactive manipulation of joint angles
- Visualization of workspace and reachable space

**Usage:**

```bash
cd Learning/2_Planar3R_Forward_Kinematics
python mj_planar3R_forward_kinematics.py
```

### 3. UR5 Forward Kinematics (Module 3)

**Location:** `Learning/3_UR5_Forward_Kinematics/`

- **Purpose:** Advanced forward kinematics with 6-DOF industrial robot
- **Key Files:**
  - `mj_forward_kinematcs.py` - MuJoCo UR5 simulation
  - `forward_kinematics.py` - Python forward kinematics implementation
  - `py_forward_kinematics.py` - Pure Python forward kinematics
  - `animate.py` - Animation utilities
  - `robot_data.py` - UR5 robot parameters and configuration
  - `utility.py` - Mathematical utility functions

**Learning Features:**

- **6-DOF UR5 robot simulation** with realistic dynamics
- **Advanced forward kinematics** using DH parameters
- **Quaternion and rotation matrix** mathematics
- **Real-time end-effector tracking** and visualization
- **Comparison between analytical and simulation** results
- **Animation capabilities** for trajectory visualization

**Key Learning Outcomes:**

- Understanding of 6-DOF robot kinematics
- DH parameter methodology
- Quaternion mathematics and conversions
- Industrial robot simulation techniques
- Advanced visualization and animation

**Usage:**

```bash
cd Learning/3_UR5_Forward_Kinematics
python mj_forward_kinematcs.py
```

### 4. UR5 Inverse Kinematics (Module 4)

**Location:** `Learning/4_UR5_Inverse_Kinematics/`

- **Purpose:** Learn inverse kinematics solving and trajectory following
- **Key Files:**
  - `mj_inverse_kinematcs.py` - Basic inverse kinematics simulation
  - `mj_inverse_kinematcs_circle.py` - Circle trajectory following
  - `inverse_kinematics.py` - Inverse kinematics solver using scipy.optimize
  - `forward_kinematics.py` - Forward kinematics for verification
  - `robot_data.py` - UR5 robot configuration
  - `utility.py` - Mathematical utilities

**Learning Features:**

- **Inverse kinematics solving** using numerical optimization
- **Trajectory following** capabilities
- **Circle path tracking** demonstration
- **Real-time IK computation** with scipy.optimize.fsolve
- **Position and orientation control** of end-effector
- **Comparison between reference and actual** end-effector poses

**Key Learning Outcomes:**

- Understanding of inverse kinematics problem formulation
- Numerical optimization techniques for robotics
- Trajectory planning and execution
- Real-time control implementation
- Error analysis and verification

**Usage:**

```bash
# Basic inverse kinematics
cd Learning/4_UR5_Inverse_Kinematics
python mj_inverse_kinematcs.py

# Circle trajectory following
python mj_inverse_kinematcs_circle.py
```

## 🤖 Robot Models

### UR5 Industrial Robot

**Location:** `Learning/Models/UR5/`

The project includes a complete UR5 robot model with:

- **6 degrees of freedom** (shoulder, elbow, wrist joints)
- **Realistic joint limits** and dynamics
- **High-quality mesh models** for visual rendering
- **Collision geometries** for physics simulation
- **Complete scene setup** with proper lighting and materials

**Model Files:**

- `ur5e.xml` - Main robot model
- `scene.xml` - Complete scene with robot
- `assets/` - 3D mesh files for all robot components

### Planar 3R Manipulator

**Location:** `Learning/1_Planar Manipulator/` and `Learning/2_Planar3R_Forward_Kinematics/`

A simplified 3-revolute joint manipulator:

- **3 degrees of freedom** in the XY plane
- **Educational focus** on basic kinematics concepts
- **Interactive visualization** with real-time position tracking

## 🛠️ Setup Instructions

### Prerequisites

- Python 3.8 or higher
- MuJoCo 3.0 or higher
- Required Python packages (see requirements.txt)

### Installation

1. **Clone or download the project**

   ```bash
   git clone <repository-url>
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
   python -c "import mujoco, numpy, scipy; print('Installation successful!')"
   ```

## 📦 Dependencies

The project uses the following main packages:

- **mujoco (≥3.0.0)** - Physics simulation engine
- **numpy (≥1.21.0)** - Numerical computations
- **scipy (≥1.7.0)** - Optimization and scientific computing
- **matplotlib (≥3.5.0)** - Plotting and visualization
- **mediapy (≥1.1.0)** - Media processing and visualization

## 🎮 Usage Examples

### Learning Path Progression

1. **Start with Planar Manipulator**

   ```bash
   cd Learning/1_Planar_Manipulator
   # Examine the XML model structure
   ```

2. **Forward Kinematics Learning**

   ```bash
   cd Learning/2_Planar3R_Forward_Kinematics
   python mj_planar3R_forward_kinematics.py
   ```

3. **Advanced Forward Kinematics**

   ```bash
   cd Learning/3_UR5_Forward_Kinematics
   python mj_forward_kinematcs.py
   ```

4. **Inverse Kinematics Mastery**
   ```bash
   cd Learning/4_UR5_Inverse_Kinematics
   python mj_inverse_kinematcs.py
   python mj_inverse_kinematcs_circle.py
   ```

### Interactive Features

Each module includes:

- **Mouse controls** for camera manipulation
- **Keyboard shortcuts** for simulation control
- **Real-time visualization** of robot motion
- **Mathematical verification** displays
- **Error analysis** and comparison tools

## 🎥 Media and Visualizations

**Available Media:**

- **Demo GIF:** [`media/demo.gif`](media/demo.gif) - General simulation demonstration
- **Demo Video:** [`media/output.mp4`](media/output.mp4) - Demo video output
- **Unstable Walking GIF:** [`media/unstable_walking.gif`](media/unstable_walking.gif) - Research locomotion patterns
- **Unstable Walking Video:** [`media/unstable_walking.mp4`](media/unstable_walking.mp4) - 15-second unstable walking research footage
- **Project Screenshot:** [`media/screenshot.png`](media/screenshot.png) - Overview of the simulation environment

## 🔧 Customization & Development

### Creating Custom Simulations

Use any module as a starting point for your own simulations:

```python
# Copy a module and modify
cp -r Learning/3_UR5_Forward_Kinematics my_custom_module

# Key areas to customize:
# 1. XML model path
# 2. Controller logic in controller() function
# 3. Initialization in init_controller() function
# 4. Camera settings for visualization
# 5. Forward/inverse kinematics calculations
```

### Simulation Parameters

Key parameters you can modify:

```python
# Simulation timing
simend = 5  # Simulation duration (seconds)
dt = 0.02   # Time step

# Camera settings
cam.azimuth = -130
cam.elevation = -5
cam.distance = 2
cam.lookat = np.array([0.0, 0.0, 0.5])

# Control parameters
ctrl_rate = 60  # Control update rate
```

## 🎓 Educational Value

This project is designed for:

- **Students** learning robotics and simulation
- **Researchers** exploring kinematics algorithms
- **Developers** building custom MuJoCo applications
- **Educators** teaching robotics concepts

### Learning Progression

1. **Module 1:** Basic robot structure and joint concepts
2. **Module 2:** Forward kinematics fundamentals with verification
3. **Module 3:** Advanced 6-DOF forward kinematics
4. **Module 4:** Inverse kinematics and trajectory following

Each module builds upon the previous one, providing a comprehensive understanding of robotic manipulation.

## 🐛 Troubleshooting

### Common Issues

1. **Import Error: No module named 'mujoco'**

   - Ensure virtual environment is activated
   - Reinstall with: `pip install mujoco>=3.0.0`

2. **OpenGL/Graphics Issues**

   - Update graphics drivers
   - Try software rendering: `export MUJOCO_GL=osmesa`

3. **Model Loading Errors**

   - Check that model XML files exist in correct paths
   - Verify all mesh files are in the assets directory

4. **Inverse Kinematics Convergence Issues**
   - Try different initial joint configurations
   - Adjust optimization parameters in scipy.optimize.fsolve

## 📊 Performance Notes

- **Simulation speed**: ~60 FPS on typical hardware
- **Memory usage**: ~200-500MB depending on module
- **Learning modules**: Optimized for educational use with clear visualizations
- **Real-time IK**: Achievable for simple trajectories

## 🤝 Contributing

This project is designed for educational purposes. Feel free to:

- Add new learning modules
- Improve existing implementations
- Create additional robot models
- Enhance visualization features

The structured learning approach makes it easy to contribute new educational content while maintaining the progressive learning path.
