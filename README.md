# 🚁 3D Drone Pathfinding AI Simulator

A 3D drone simulation built with OpenGL that lets you fly a drone around obstacles in real-time. Think of it like a video game where you control a drone and need to navigate through a city with skyscrapers and mountains.

## 🎯 What This Project Does

This simulator creates a 3D world where you can:
- **Fly a drone** using keyboard controls (WASD, QE, RF, ZX)
- **Switch camera views** between first-person (you're the drone) and third-person (following the drone)
- **Navigate obstacles** like skyscrapers, mountains, and ground obstacles
- **Experience realistic physics** - the drone has momentum, drag, and realistic movement

## 🏗️ How It's Built (The Big Picture)

### Why OpenGL Instead of OpenCV?
Originally, this project used OpenCV (a computer vision library) to draw 3D graphics. But OpenCV is designed for image processing, not 3D graphics. It was like trying to paint a 3D sculpture with a 2D paintbrush.

**OpenGL** is the right tool because:
- It's designed specifically for 3D graphics
- It uses your computer's graphics card for fast rendering
- It handles 3D math (like perspective, depth, lighting) automatically
- It's what real 3D games use

### The Main Parts (Like Building Blocks)

Think of this project like building a house with different rooms:

1. **Renderer** (`src/Renderer.cpp`) - The "artist" that draws everything
2. **Drone** (`src/Drone.cpp`) - The "physics engine" for the drone
3. **Camera** (`src/Camera.cpp`) - The "eyes" that determine what you see
4. **World** (`src/World.cpp`) - The "city planner" that creates obstacles
5. **InputHandler** (`src/InputHandler.cpp`) - The "controller" that reads your keyboard/mouse

## 🔧 How Each Part Works

### 1. Renderer - The 3D Artist

**What it does:** Draws everything you see on screen

**How it works:**
- **Shaders** are like "paint recipes" that tell the graphics card how to color each pixel
- **Vertices** are the corners of 3D objects (like the corners of a cube)
- **Matrices** are math tools that handle 3D positioning and perspective

**Why it's complex:** 3D graphics require lots of math to convert 3D coordinates into 2D screen pixels. The Renderer handles all this math automatically.

### 2. Drone - The Physics Engine

**What it does:** Makes the drone move realistically

**How it works:**
- **State** = where the drone is (x, y, z) and how it's oriented (roll, pitch, yaw)
- **Input** = what buttons you're pressing
- **Physics update** = calculates new position based on input and physics laws

**Why physics matter:** Without physics, the drone would move instantly from point A to point B. With physics, it accelerates, has momentum, and feels realistic to control.

**The math behind it:**
```
New Position = Old Position + (Velocity × Time)
New Velocity = Old Velocity + (Acceleration × Time)
```

### 3. Camera - Your Viewpoint

**What it does:** Determines what you see on screen

**Two modes:**
- **First-person:** You're inside the drone, seeing what the drone sees
- **Third-person:** You're following the drone from behind, like a camera operator

**How it works:**
- **Position:** Where the camera is located
- **Target:** What the camera is looking at
- **Up vector:** Which way is "up" (prevents the world from appearing sideways)

**Why two modes:** First-person gives you precise control, third-person gives you better awareness of your surroundings.

### 4. World - The Environment

**What it does:** Creates the obstacles and terrain

**Types of obstacles:**
- **Skyscrapers:** Tall buildings that block your path
- **Mountains:** Natural obstacles with height
- **Ground obstacles:** Smaller objects on the ground

**How it works:**
- Randomly generates obstacles in different locations
- Each obstacle has a type, position, and size
- The Renderer draws each obstacle as a 3D cube

### 5. InputHandler - The Controller

**What it does:** Reads your keyboard and mouse input

**How it works:**
- **Key press/release:** Detects when you press or release keys
- **Mouse movement:** Tracks mouse position for camera control
- **Input mapping:** Converts key presses into drone commands

**Why it's separate:** This keeps the drone physics separate from input handling, making the code easier to understand and modify.

## 🎮 Controls Explained

### Movement Controls
- **W/S:** Forward/Backward thrust (like gas pedal and brake)
- **A/D:** Roll left/right (tilting the drone sideways)
- **Q/E:** Yaw left/right (turning the drone left or right)
- **R/F:** Pitch up/down (pointing the drone up or down)
- **Z/X:** Vertical thrust (going up or down)

### Camera Controls
- **Space:** Switch between first-person and third-person views
- **Mouse:** Look around (first-person) or orbit around drone (third-person)
- **ESC:** Exit the simulation

## 🔬 The Technical Deep Dive (Simplified)

### Why 3D Math is Hard

Imagine you're trying to draw a 3D cube on a 2D piece of paper. You need to:
1. Figure out where each corner of the cube is in 3D space
2. Calculate how far away each corner is from the viewer
3. Project those 3D points onto the 2D paper
4. Handle perspective (things farther away look smaller)

This is what the **projection matrix** does - it's like a mathematical lens that converts 3D coordinates to 2D screen coordinates.

### Why Shaders Matter

**Shaders** are small programs that run on your graphics card. They're like "paint brushes" that determine:
- **Vertex shader:** Where each point of a 3D object goes on screen
- **Fragment shader:** What color each pixel should be

Without shaders, everything would look like flat, unlit shapes. With shaders, we get:
- Realistic lighting (shadows, highlights)
- Smooth colors and textures
- 3D depth perception

### The Game Loop

The simulation runs in a continuous loop:

```
1. Read input (keyboard/mouse)
2. Update drone physics
3. Update camera position
4. Clear the screen
5. Draw the 3D world
6. Show the result
7. Repeat
```

This happens 60+ times per second, creating the illusion of smooth movement.

## 🚀 How to Build and Run

### Prerequisites
- **macOS** (tested on macOS 14)
- **CMake** (build system)
- **OpenGL** (3D graphics)
- **GLFW** (window management)
- **GLM** (math library)

### Build Steps
```bash
mkdir build
cd build
cmake ..
make
./drone_sim
```

### Why These Libraries?

- **CMake:** Automatically finds and links all the required libraries
- **OpenGL:** Provides 3D graphics capabilities
- **GLFW:** Creates windows and handles input (keyboard/mouse)
- **GLM:** Provides math functions for 3D calculations (vectors, matrices)

## 🐛 Common Issues and Solutions

### "Shader files not found"
- **Problem:** The program can't find the shader files
- **Solution:** Make sure you're running from the `build` directory (shaders are in `../shaders/`)

### "OpenGL deprecated warnings"
- **Problem:** macOS shows warnings about OpenGL being deprecated
- **Solution:** These are just warnings - the program still works. OpenGL is being replaced by Metal on macOS, but OpenGL still works fine.

### "No 3D window appears"
- **Problem:** Program runs but no graphics window shows
- **Solution:** Check that all libraries are properly linked and shaders are loading

## 🔮 Future Improvements

### What Could Be Added
1. **Collision detection** - Prevent drone from flying through obstacles
2. **Pathfinding AI** - Make the drone find its own way around obstacles
3. **More realistic physics** - Add wind, turbulence, battery life
4. **Better graphics** - Textures, shadows, particle effects
5. **Sound effects** - Engine noise, wind, collision sounds

### Why These Would Be Cool
- **Collision detection** would make the simulation more realistic
- **Pathfinding AI** would demonstrate actual AI behavior
- **Better physics** would make flying feel more authentic
- **Graphics improvements** would make it look more professional

## 🎓 What You'll Learn

By studying this code, you'll understand:

1. **3D Graphics Programming** - How 3D worlds are created and rendered
2. **Game Development** - How game loops, physics, and input work
3. **C++ Programming** - Object-oriented design, memory management, libraries
4. **Mathematics** - 3D vectors, matrices, transformations
5. **Software Architecture** - How to organize code into logical components

## 🤔 Why This Architecture?

### Separation of Concerns
Each class has one job:
- **Renderer** only draws things
- **Drone** only handles physics
- **Camera** only manages viewpoint
- **World** only creates obstacles
- **InputHandler** only reads input

This makes the code:
- **Easier to understand** - you can look at one part without understanding everything
- **Easier to modify** - change the physics without touching the graphics
- **Easier to test** - test each component separately
- **Easier to reuse** - use the physics engine in other projects

### Data Flow
```
Input → InputHandler → Drone → Camera → Renderer → Screen
```

Each component passes data to the next, creating a clear flow of information.

## 📚 Key Concepts Summary

1. **3D Graphics** = Converting 3D objects to 2D screen pixels
2. **Physics** = Making movement feel realistic with acceleration and momentum
3. **Game Loop** = Continuous cycle of input → update → render
4. **Shaders** = Programs that run on your graphics card to create visual effects
5. **Matrices** = Math tools for handling 3D positioning and perspective
6. **Separation of Concerns** = Each part of the code has one specific job

## 🎉 Conclusion

This project demonstrates how to build a complete 3D simulation from scratch. It shows how different programming concepts (3D graphics, physics, input handling, software architecture) come together to create an interactive experience.

The key insight is that complex systems are built by combining simple, well-designed components. Each piece does one thing well, and together they create something much more powerful than any individual part.

Happy flying! 🚁✈️
