# ROS Applications

This repository contains the official material for the **ROS Applications** course.  
It provides practical examples, tutorials, launch files, and documentation for a wide range of ROS 2 concepts, from basic visualization to advanced coordinate transformations.

The goal of this repository is to offer a clean, modular, and scalable structure for learning and teaching ROS 2 through real-world applications.

---

## Course Modules

This repository will grow into a complete collection of ROS 2 applications.  
Current and upcoming modules include:

### TF2 and Coordinate Transformations
- Introduction to TF2  
- Understanding frames and transformations  
- Static broadcasters  
- Dynamic broadcasters  
- TF2 listeners and lookupTransform  
- Adding frames (fixed and dynamic)  
- Mathematical intuition behind transformations  
- Visualizing TF2 in RViz2  

### RViz2 Visualization
- RViz2 basics  
- Fixed frame configuration  
- TF display  
- Visualizing robots, sensors, and transforms  

### Future Modules
- ROS 2 communication patterns  
- URDF and robot modeling  
- Sensors and perception  
- Navigation and control  
- MoveIt and manipulation  
- Simulation environments  

---

## Repository Structure

This repository **does not include a ROS 2 workspace**.  
Only the course packages are provided.  
Each student must create their own workspace locally.

```text
ros_applications/
├── README.md
├── LICENSE
├── .gitignore
├── packages/
│   └── learning_tf2_py/
│       ├── package.xml
│       ├── setup.py
│       ├── learning_tf2_py/
│       └── launch/
└── docs/
    ├── images/
    └── diagrams/
```

---

## Creating Your Workspace

Students must create their own ROS 2 workspace.  
The recommended workspace name for this course is **ras_ws**.

### 1. Create the workspace

```bash
mkdir -p ras_ws/src
cd ras_ws/src
```

### 2. Clone this repository

```
git clone https://github.com/gdiaz-guevara/ros_applications.git
```

### 3. Move the course packages into your workspace

```
mv ros_applications/packages/* .
```

### 4. Build the workspace

```bash
cd ..
colcon build
source install/setup.bash
```

---

## Documentation (Wiki)

The full documentation for each module is available in the repository’s Wiki.  
It includes step‑by‑step guides, code explanations, mathematical intuition, diagrams, and exercises.

Recommended reading order:

1. Introduction to TF2  
2. RViz2 basics  
3. Static broadcaster  
4. Dynamic broadcaster  
5. Listener and lookupTransform  
6. Frames and transformations  
7. Adding frames  
8. Mathematical intuition behind TF2  

---

## Contributing

Contributions are welcome.  
Feel free to open issues, submit pull requests, or propose new modules.

---

## License

This project is licensed under the MIT License.
