# Lesson 1: Installing Isaac Sim and Understanding the Ecosystem

Isaac Sim runs on NVIDIA's Omniverse platform—a collaborative 3D design ecosystem powered by USD (Universal Scene Description). Unlike standalone simulators like Gazebo, Isaac Sim is an **Omniverse application** that leverages GPU-accelerated rendering, physics, and AI capabilities.

This lesson walks you through installation, GPU verification, and ecosystem exploration. You'll learn by doing: install the tools, run sample scenes, and manually explore the interface to build intuition before AI assistance in later lessons.

## Prerequisites Check

Before beginning installation, verify your system meets these requirements:

**Hardware requirements**:
- NVIDIA GPU with 8GB+ VRAM (minimum: GTX 1080 / RTX 2070)
  - **Recommended**: RTX 3070 or better
  - **Optimal**: RTX 4070/4080/4090
- 32GB+ RAM (Isaac Sim loads large scenes into memory)
- 100GB+ free disk space (Isaac Sim + assets)
- Ubuntu 22.04 LTS (native installation strongly recommended)

**Software requirements**:
- NVIDIA GPU drivers 535+ (for CUDA 12.x support)
- CUDA Toolkit 12.x (will be installed automatically)
- Vulkan support (for RTX rendering)

**Check your GPU**:
```bash
nvidia-smi
```

**Expected output**:
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.54.03    Driver Version: 535.54.03    CUDA Version: 12.2   |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  NVIDIA GeForce ...  Off  | 00000000:01:00.0  On |                  N/A |
| 30%   45C    P8    12W / 220W |   1024MiB / 12288MiB |      2%      Default |
+-------------------------------+----------------------+----------------------+
```

**Critical check**: Driver version must be 535+ and CUDA version 12.0+. If your driver is older, update before proceeding.

**Update NVIDIA drivers (if needed)**:
```bash
# Add NVIDIA driver PPA
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# Install driver 535 or newer
sudo apt install nvidia-driver-535
sudo reboot

# Verify after reboot
nvidia-smi
```

## Understanding Omniverse Architecture

Before installation, understand what you're installing:

**Omniverse Launcher**: Application hub for managing NVIDIA tools
- Downloads and updates Omniverse applications
- Manages Nucleus (asset collaboration server)
- Handles authentication and licensing

**Isaac Sim**: Robotics simulation application
- Built on Omniverse Kit (extensible application framework)
- Uses PhysX 5 for GPU-accelerated physics
- Uses RTX for photorealistic ray tracing
- Includes Replicator API for synthetic data generation

**Omniverse Nucleus**: Local or cloud asset storage
- USD file management and versioning
- Collaborative editing (multiple users, same scene)
- Asset streaming for large scenes

**Architecture diagram** (conceptual):
```
Omniverse Launcher (manages):
├── Isaac Sim (application)
│   ├── Omniverse Kit (framework)
│   ├── PhysX 5 (physics)
│   ├── RTX Rendering (graphics)
│   └── Replicator (synthetic data)
└── Nucleus (asset server)
    └── USD files (scenes, robots, materials)
```

## Installing Omniverse Launcher

**Step 1: Download Omniverse Launcher**

Visit: https://www.nvidia.com/en-us/omniverse/download/

**Important**: Select **Linux** version (Ubuntu 22.04).

```bash
# Download will be named something like:
# omniverse-launcher-linux.AppImage

# Make executable
chmod +x omniverse-launcher-linux.AppImage

# Run launcher
./omniverse-launcher-linux.AppImage
```

**Step 2: Create NVIDIA Account (Free)**

On first launch, you'll be prompted to sign in or create an NVIDIA account.

**License note**: Isaac Sim is **free for education, research, and personal use**. No payment required for learning purposes.

**Step 3: Install Nucleus (Local Server)**

In Omniverse Launcher:
1. Go to "Nucleus" tab
2. Click "Install" for **Nucleus Workstation** (local server)
3. Accept default installation path
4. Wait for installation (5-10 minutes)

**Why Nucleus?** Even for solo learning, Nucleus provides USD asset management. You'll store robot models, scenes, and materials here.

**Verify Nucleus running**:
```bash
# Nucleus runs as a local service
systemctl --user status omniverse-nucleus
```

**Expected output**: Service should show "active (running)".

## Installing Isaac Sim

**Step 4: Install Isaac Sim Application**

In Omniverse Launcher:
1. Go to "Exchange" tab
2. Search for **"Isaac Sim"**
3. Click "Install" (latest version: 2023.1.1 or newer)
4. Select installation path (requires ~80GB free space)
5. Wait for download and installation (30-60 minutes depending on internet speed)

**Installation contents**:
- Isaac Sim application (~40GB)
- Sample assets and scenes (~20GB)
- Python dependencies and libraries (~10GB)
- PhysX and RTX binaries (~10GB)

**Step 5: Launch Isaac Sim**

In Omniverse Launcher:
1. Go to "Library" tab
2. Click "Launch" next to **Isaac Sim**
3. First launch is slow (shader compilation, cache building)
4. Wait 2-5 minutes for startup

**Expected**: Isaac Sim opens with welcome screen and empty viewport.

## Verifying GPU Acceleration

**Critical validation**: Ensure Isaac Sim is using your GPU, not CPU fallback.

**Step 6: Run Benchmark Scene**

In Isaac Sim:
1. Menu: **Isaac Examples → Manipulation → Pick and Place**
2. Scene loads (robot arm, table, objects)
3. Click **Play** button (bottom-left)
4. Observe physics simulation running

**Performance check**:
```bash
# In separate terminal, monitor GPU usage
watch -n 1 nvidia-smi
```

**What to check**:
- **GPU-Util**: Should be 40-80% during simulation
- **Memory-Usage**: Should increase (scene loaded into VRAM)
- **Temperature**: Should rise moderately (GPU active)

**If GPU-Util stays at 0-5%**: GPU acceleration may not be working. See troubleshooting section.

**Step 7: Verify RTX Ray Tracing**

In Isaac Sim viewport:
1. Click **Render Settings** (icon looks like a light bulb)
2. Under **Render Mode**, ensure **"RTX - Interactive (Path Tracing)"** is selected
3. Observe realistic shadows and reflections

**Compare render modes**:
- **RTX Path Tracing**: Photorealistic, accurate shadows, slower (20-60 FPS)
- **Rasterization**: Faster but less realistic (60-120 FPS)

**Toggle between modes** to see the difference. RTX mode should show soft shadows and accurate reflections.

## Exploring the Isaac Sim Interface

**Manual exploration exercise**: Spend 10-15 minutes exploring these interface elements.

**Key panels** (identify each):

1. **Viewport** (center):
   - 3D scene visualization
   - Navigation: Middle-click drag (rotate), Scroll (zoom), Shift+middle-click (pan)

2. **Stage** (left panel):
   - USD hierarchy (prims tree)
   - Expand to see scene objects
   - Similar to Gazebo's model tree

3. **Property** (right panel):
   - Selected object attributes
   - Physics properties
   - Rendering settings

4. **Content Browser** (bottom):
   - Asset library navigation
   - Nucleus paths (localhost/NVIDIA/Assets)
   - Drag-and-drop to add assets

5. **Timeline** (bottom-left):
   - Play/Pause/Stop simulation
   - Timeline scrubbing
   - FPS display

**Exercise 1: Load a Sample Scene**

1. Menu: **Isaac Examples → Robots → Humanoid**
2. Humanoid robot loads in viewport
3. Click **Play** → Robot should stand stable
4. In **Stage** panel, expand humanoid prim
5. Observe USD hierarchy (links, joints, collisions)

**Exercise 2: Navigate the Viewport**

Practice camera controls:
- **Rotate**: Middle-click and drag
- **Pan**: Shift + middle-click and drag
- **Zoom**: Scroll wheel
- **Frame selected**: Press **F** (frames viewport on selected object)

**Exercise 3: Inspect Object Properties**

1. In viewport, click humanoid's torso
2. Property panel shows selected link properties
3. Scroll to **Physics → Rigid Body** section
4. Note: Mass, Center of Mass, Inertia values

Compare to Gazebo: Properties are similar but PhysX uses different units and parameters.

## System Requirements Validation

**Validate your system can handle Isaac Sim workloads**:

**GPU memory test**:
```python
# In Isaac Sim, open Script Editor (Window → Script Editor)
# Run this Python code:

import carb

# Get GPU memory stats
gpu_stats = carb.graphics.get_gpu_stats()
print(f"Total VRAM: {gpu_stats['total_memory'] / 1024 / 1024:.2f} MB")
print(f"Used VRAM: {gpu_stats['used_memory'] / 1024 / 1024:.2f} MB")
print(f"Free VRAM: {gpu_stats['free_memory'] / 1024 / 1024:.2f} MB")
```

**Expected output**:
```
Total VRAM: 12288.00 MB
Used VRAM: 3456.78 MB
Free VRAM: 8831.22 MB
```

**Interpretation**:
- **8GB+ VRAM**: Sufficient for moderate scenes
- **12GB+ VRAM**: Good for complex scenes with domain randomization
- **16GB+ VRAM**: Optimal for large-scale synthetic data generation

**If free VRAM < 4GB**: Close other GPU applications (browsers, other simulations) before running Isaac Sim.

## Common Installation Issues

**Issue 1: NVIDIA driver mismatch**

**Symptom**: Isaac Sim won't launch or crashes on startup.

**Fix**:
```bash
# Check driver version
nvidia-smi

# If driver < 535, update:
sudo apt install nvidia-driver-535
sudo reboot
```

**Issue 2: Vulkan not supported**

**Symptom**: Error message about missing Vulkan libraries.

**Fix**:
```bash
# Install Vulkan runtime
sudo apt install vulkan-tools libvulkan1

# Test Vulkan
vulkaninfo | grep "Vulkan Instance Version"
```

**Expected**: Should show Vulkan version 1.2 or higher.

**Issue 3: Insufficient VRAM**

**Symptom**: Isaac Sim launches but crashes when loading scenes.

**Fix**:
- Close other GPU applications
- Use simpler scenes (fewer objects)
- Reduce render quality (Settings → Render Settings → Quality: Low)

**Issue 4: Slow startup (5+ minutes)**

**Symptom**: Isaac Sim takes very long to launch.

**This is normal on first launch** (shader compilation). Subsequent launches should be faster (1-2 minutes).

**Issue 5: No GPU acceleration (GPU-Util = 0%)**

**Symptom**: Simulation runs but GPU utilization stays at 0-5%.

**Check**:
```bash
# Ensure CUDA is accessible
nvidia-smi

# Verify Isaac Sim can see GPU
# In Isaac Sim Script Editor:
import carb
print(carb.graphics.get_active_gpu())
```

**Expected output**: Should print GPU name (e.g., "NVIDIA GeForce RTX 4070").

**If GPU not detected**: Check NVIDIA drivers and CUDA installation.

## Understanding Isaac Sim Startup

**What happens during launch** (for debugging understanding):

1. **Omniverse Kit initialization** (5-10 seconds)
   - Load extensions
   - Initialize rendering pipeline

2. **PhysX 5 startup** (5 seconds)
   - Initialize GPU-accelerated physics
   - Load physics engine configuration

3. **RTX rendering pipeline** (10-20 seconds)
   - Compile shaders (first launch only)
   - Initialize ray tracing kernel

4. **Extension loading** (10-15 seconds)
   - Isaac Sim robotics extensions
   - ROS 2 bridge (if installed)
   - Replicator API

5. **Nucleus connection** (2-5 seconds)
   - Connect to local Nucleus server
   - Mount asset paths

**Total first launch**: 2-5 minutes (normal)
**Subsequent launches**: 1-2 minutes

## Validation Checkpoint

Before proceeding to Lesson 2, verify:

- [ ] **Omniverse Launcher installed and running**
- [ ] **Nucleus Workstation installed and active**
- [ ] **Isaac Sim installed (version 2023.1.1+)**
- [ ] **Isaac Sim launches successfully (within 5 minutes)**
- [ ] **GPU acceleration verified** (nvidia-smi shows GPU activity during simulation)
- [ ] **RTX rendering active** (can toggle RTX mode and see ray-traced shadows)
- [ ] **Sample scene loaded and runs** (humanoid stands stable when playing)
- [ ] **Viewport navigation working** (can rotate, pan, zoom camera)
- [ ] **Stage and Property panels accessible** (can inspect object hierarchy)

**All items checked?** You're ready for Lesson 2 (Isaac Sim Architecture and USD Format).

**Installation issues persisting?** Consider cloud alternatives (AWS EC2 g4dn instances with Isaac Sim pre-installed) or reach out to NVIDIA Omniverse forums for community support.

## Try With AI

**Setup**: Open ChatGPT or Claude.

**Prompt 1: Troubleshooting Installation Issues**
```
I'm installing Isaac Sim on Ubuntu 22.04. I have an NVIDIA RTX 4070 GPU. What are the most common installation pitfalls and how do I verify each component is working correctly?
```

**Prompt 2: Optimizing for Your Hardware**
```
My system has:
- NVIDIA RTX 3070 (8GB VRAM)
- 16GB RAM
- Ubuntu 22.04

What Isaac Sim render settings should I use for a good balance between visual quality and performance when generating synthetic training data?
```

**Prompt 3: Understanding Omniverse Ecosystem**
```
I'm new to NVIDIA Omniverse. Explain the relationship between:
1. Omniverse Launcher
2. Nucleus
3. Isaac Sim
4. USD files

How do these components work together for robotics simulation?
```

**Expected outcomes**: AI will provide troubleshooting strategies, performance tuning recommendations, and conceptual explanations. Use these to deepen your understanding of the ecosystem architecture.

**Safety note**: Always verify commands that modify system settings (driver installation, package updates) before executing. Backup important data before major system changes.
