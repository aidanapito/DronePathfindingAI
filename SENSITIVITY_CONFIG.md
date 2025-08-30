# 🎮 Sensitivity Configuration Guide

This guide shows you how to adjust various sensitivity settings in your drone simulator to make the controls feel just right.

## 📍 **Current Settings (After Fixes)**

### ✅ **Fixed Issues:**

1. **A/D controls are now correct** - A rolls left, D rolls right
2. **Q/E controls are now correct** - Q yaws left, E yaws right  
3. **Camera movement is much slower** - Reduced from 0.01f to 0.002f (80% slower!)
4. **Zooming is much slower** - Reduced from 10.0f to 2.0f (80% slower!)
5. **Mouse jitter reduced** - Added deadzone to prevent tiny movements
6. **Space key camera switching now works** - Fixed camera mode update logic
7. **A/D camera roll effect reduced** - Added camera roll sensitivity control (30% of drone roll)

## 🔧 **How to Adjust Sensitivity**

### **Camera Movement Speed**
**File:** `include/InputHandler.h` and `include/Camera.h`

```cpp
// Current setting (much slower)
static constexpr float ORBIT_SENSITIVITY = 0.002f;

// Make it even slower
static constexpr float ORBIT_SENSITIVITY = 0.001f;

// Make it faster
static constexpr float ORBIT_SENSITIVITY = 0.005f;

// Make it much faster (original)
static constexpr float ORBIT_SENSITIVITY = 0.01f;
```

**What this affects:**
- How fast the camera moves when you move your mouse
- Lower values = slower, more controlled camera movement
- Higher values = faster, more responsive camera movement

### **Zoom Speed**
**File:** `include/InputHandler.h` and `include/Camera.h`

```cpp
// Current setting (much slower)
static constexpr float ZOOM_SENSITIVITY = 2.0f;

// Make it even slower
static constexpr float ZOOM_SENSITIVITY = 1.0f;

// Make it faster
static constexpr float ZOOM_SENSITIVITY = 5.0f;

// Make it much faster (original)
static constexpr float ZOOM_SENSITIVITY = 10.0f;
```

### **Camera Roll Sensitivity (A/D Key Effect)**
**File:** `include/Camera.h`

```cpp
// Current setting (reduced effect)
static constexpr float CAMERA_ROLL_SENSITIVITY = 0.3f;

// Make A/D have no camera effect
static constexpr float CAMERA_ROLL_SENSITIVITY = 0.0f;

// Make A/D have full camera effect
static constexpr float CAMERA_ROLL_SENSITIVITY = 1.0f;

// Make A/D have very subtle camera effect
static constexpr float CAMERA_ROLL_SENSITIVITY = 0.1f;
```

**What this affects:**
- How fast the camera zooms in/out
- Lower values = slower zooming
- Higher values = faster zooming

### **Mouse Deadzone**
**File:** `include/InputHandler.h`

```cpp
// Current setting
static constexpr float MOUSE_DEADZONE = 1.0f;

// Make it more sensitive (smaller deadzone)
static constexpr float MOUSE_DEADZONE = 0.5f;

// Make it less sensitive (larger deadzone)
static constexpr float MOUSE_DEADZONE = 2.0f;
```

**What this affects:**
- How much you need to move your mouse before the camera responds
- Lower values = more sensitive (might cause jitter)
- Higher values = less sensitive (more stable, but less responsive)

## 🚁 **Drone Movement Sensitivity (Future Enhancement)**

These constants are ready for when you want to adjust drone movement speed:

```cpp
// In include/InputHandler.h
static constexpr float DRONE_ROLL_SENSITIVITY = 1.0f;   // Roll speed (A/D keys)
static constexpr float DRONE_YAW_SENSITIVITY = 1.0f;    // Yaw speed (Q/E keys)
static constexpr float DRONE_PITCH_SENSITIVITY = 1.0f;  // Pitch speed (R/F keys)
```

**To use these:**
1. Change the values in `InputHandler.cpp` where drone inputs are processed
2. Multiply the input values by these sensitivity constants

## 🎯 **Recommended Settings for Different Preferences**

### **For Beginners (Very Slow, Controlled Movement)**
```cpp
static constexpr float ORBIT_SENSITIVITY = 0.001f;    // Extremely slow camera
static constexpr float ZOOM_SENSITIVITY = 1.0f;       // Very slow zoom
static constexpr float MOUSE_DEADZONE = 2.0f;         // Large deadzone
static constexpr float CAMERA_ROLL_SENSITIVITY = 0.1f; // Minimal camera roll effect
```

### **For Experienced Users (Fast, Responsive Movement)**
```cpp
static constexpr float ORBIT_SENSITIVITY = 0.008f;    // Fast camera
static constexpr float ZOOM_SENSITIVITY = 8.0f;       // Fast zoom
static constexpr float MOUSE_DEADZONE = 0.5f;         // Small deadzone
```

### **For Precision Flying (Current Setting)**
```cpp
static constexpr float ORBIT_SENSITIVITY = 0.002f;    // Current setting
static constexpr float ZOOM_SENSITIVITY = 2.0f;       // Current setting
static constexpr float MOUSE_DEADZONE = 1.0f;         // Current setting
static constexpr float CAMERA_ROLL_SENSITIVITY = 0.3f; // Current setting
```

## 🔄 **How to Apply Changes**

1. **Edit the constants** in `include/InputHandler.h` and `include/Camera.h`
2. **Rebuild the project:**
   ```bash
   cd build
   make
   ```
3. **Test the new settings**
4. **Adjust as needed** until it feels right

## 💡 **Pro Tips**

- **Start with small changes** - Change sensitivity by 0.001f at a time
- **Test in both camera modes** - First-person and third-person
- **Consider your mouse DPI** - High DPI mice might need lower sensitivity
- **Balance responsiveness vs. stability** - Too sensitive = jittery, too slow = unresponsive

## 🐛 **Troubleshooting**

### **Camera moves too fast:**
- Reduce `ORBIT_SENSITIVITY` (try 0.003f or 0.002f)

### **Camera moves too slow:**
- Increase `ORBIT_SENSITIVITY` (try 0.008f or 0.01f)

### **Mouse feels jittery:**
- Increase `MOUSE_DEADZONE` (try 1.5f or 2.0f)

### **Zooming is too fast/slow:**
- Adjust `ZOOM_SENSITIVITY` accordingly

## 🎮 **Current Control Scheme (After Fixes)**

- **W/S:** Forward/Backward ✅
- **A/D:** Roll Left/Right ✅ (Fixed!)
- **Q/E:** Yaw Left/Right ✅
- **R/F:** Pitch Up/Down ✅
- **Z/X:** Fly Up/Down ✅
- **Mouse:** Camera control (slower, more controlled) ✅
- **Space:** Switch camera mode ✅
- **ESC:** Exit ✅

Happy flying with your newly calibrated controls! 🚁✈️
