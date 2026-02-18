# HoloAssist  
## XR-Based Human–Robot Collaboration Framework

HoloAssist is a real-time Extended Reality (XR) teleoperation and collaboration framework that integrates immersive spatial interfaces with a physical collaborative robot.

The system enables intuitive human–robot collaboration by combining:

- XR visual overlays  
- Real-time robot control  
- RGB-D perception  
- Dynamic safety constraints  
- Point cloud workspace modelling  

The project explores how immersive spatial interfaces can improve safety, flexibility, and usability in human–robot interaction (HRI).

---

## Project Overview

HoloAssist integrates a Meta Quest 2/3 headset with a collaborative robot system to enable:

- Real-time teleoperation via XR controllers  
- Freeform end-effector control  
- Visualisation of robot trajectories in 3D space  
- Live RGB-D point cloud mapping  
- Dynamic no-go zones based on workspace perception  
- Human-in-the-loop assistance for real-world tasks  

Example collaborative tasks include:

- Assisting with screw driving  
- Passing tools  
- Holding components in place  
- Semi-autonomous task execution with supervision  

---

## Core Concepts

This project bridges:

- Human–Robot Collaboration (HRC)  
- Visual Servoing  
- Reactive Control  
- XR-based Teleoperation  
- Point Cloud Processing  
- Spatial Interface Design  

We operate under both:

- **Sense–Act–Repeat** (reactive control)  
- **Sense–Think–Act** (planned trajectories)  

---

## System Architecture

### Subsystems

### XR Interface (Meta Quest 2/3)

- Controller input capture  
- Spatial pose tracking  
- Virtual robot visualisation  
- Trajectory overlays  
- No-go zone visualisation  

### Perception Layer

- RGB-D camera  
- Point cloud generation  
- Object detection and segmentation  
- Dynamic obstacle detection  

### Planning Layer

- Trajectory generation  
- Collision checking  
- Workspace constraint updates  
- Dynamic no-go zone enforcement  

### Control Layer

- Cartesian velocity control  
- Joint trajectory execution  
- Safety-limited teleoperation  
- Hybrid freeform and constrained motion  

### Robot Hardware

- Collaborative robot arm  
- End-effector tooling  
- Safety sensors  

---

## Teleoperation Modes

### 1. Freeform Mode

Direct end-effector velocity control via XR controllers.

Used for:

- Manual manipulation  
- Fine alignment  
- Rapid prototyping  
- Demonstration tasks  

### 2. Assisted Mode

Operator defines intent while the system enforces constraints.

Includes:

- Dynamic no-go zones  
- Trajectory smoothing  
- Collision avoidance  
- Velocity limits  

### 3. Training Mode

XR-based teleoperation demonstrations used to:

- Record trajectories  
- Generate replayable motion sequences  
- Compare manual vs planned performance  

---

## Perception

The system uses an RGB-D camera to:

- Generate live point clouds  
- Detect obstacles  
- Identify workspace geometry  
- Update dynamic collision maps  

Point cloud data is visualised in XR to enhance spatial awareness and improve operator decision-making.

---

## Safety Features

- Dynamic no-go zones  
- Velocity limiting  
- Workspace boundary enforcement  
- Human proximity awareness  
- Visual overlays for collision risk  

Safety is prioritised for real-world collaborative deployment.

---

## Hardware

- Meta Quest 2 / Meta Quest 3  
- RGB-D camera  
- Collaborative robot arm  
- XR controllers  

---

## Research Objectives

- Evaluate usability of XR for teleoperation  
- Compare reactive versus planned control strategies  
- Analyse safety improvements using dynamic spatial overlays  
- Investigate human-in-the-loop robotic collaboration  

---

## Repository Structure

```

holoassist/
│
├── xr_interface/
├── perception/
├── planning/
├── control/
├── robot_bringup/
├── docs/
└── experiments/

```

---

## Future Extensions

- Force feedback integration  
- Hybrid force-motion control  
- Multi-robot XR coordination  
- AI-assisted intent prediction  
- Learning from Demonstration  
