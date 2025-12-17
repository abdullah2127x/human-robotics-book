# Lesson 4: Photorealistic Rendering with RTX

Your humanoid now stands stable in Isaac Sim. But the default gray materials and basic lighting won't fool perception models. For synthetic training data to transfer to real robots, rendering must be **photorealistic**—accurate materials, realistic lighting, proper shadows.

This lesson teaches RTX (ray tracing) rendering through hands-on scene creation, with AI collaboration to refine material selection and lighting setup.

## Why Photorealism Matters for Sim-to-Real

**The challenge**: Models trained on unrealistic synthetic data fail on real robots.

**Example failure**:
- **Synthetic training**: Flat lighting, uniform materials, no shadows
- **Real deployment**: Warehouse with mixed lighting (overhead fluorescents + windows), varied materials (cardboard boxes, metal shelves, concrete floor)
- **Result**: Model can't detect objects it trained on because lighting/materials don't match

**Solution**: RTX ray tracing simulates how light actually behaves—bouncing off surfaces, creating soft shadows, producing realistic reflections.

## Prerequisites: Understanding PBR

**PBR (Physically-Based Rendering)** describes materials using physical properties:

**Core PBR parameters**:
1. **Albedo (Base Color)**: Surface color without lighting
2. **Roughness**: How smooth or matte the surface is (0=mirror, 1=matte)
3. **Metallic**: Whether surface is metallic (0=dielectric like plastic, 1=metal)
4. **Normal map**: Surface detail without geometry (optional)

**Why PBR**: These parameters remain constant under any lighting—the material "looks right" whether in office lighting or warehouse lighting.

## Creating an Indoor Scene

Let's build a photorealistic indoor environment where your humanoid will operate.

### Step 1: Add Ground Plane with Realistic Material

**Create floor**:
1. In Isaac Sim: **Create → Mesh → Plane**
2. Scale to 10m x 10m (large room)
3. Select plane in Stage panel

**Apply concrete floor material**:

In **Property** panel → **Material**:
- Click **"+ Add"** → **OmniPBR**
- **Albedo**: RGB(0.45, 0.45, 0.4) (concrete gray)
- **Roughness**: 0.7 (slightly rough, not glossy)
- **Metallic**: 0.0 (concrete is dielectric)

**Alternative (use material library)**:
1. **Content Browser** → `localhost/NVIDIA/Materials/vMaterials_2/Ground`
2. Drag **Concrete_Aged_01** onto plane
3. Observe realistic texture appears

### Step 2: Add Walls and Ceiling

**Create room enclosure**:
```python
from pxr import Usd, UsdGeom, UsdShade
from omni.isaac.core.utils.prims import create_prim
import omni.isaac.core.utils.numpy.rotations as rot_utils

# Helper to create wall with material
def create_wall(stage, path, size, position, rotation_euler):
    # Create cube
    cube_prim = create_prim(
        path,
        "Cube",
        position=position,
        scale=size
    )

    # Apply rotation
    xform = UsdGeom.Xformable(cube_prim)
    rot_op = xform.AddRotateXYZOp()
    rot_op.Set(rotation_euler)

    # Apply material (painted wall)
    material_path = f"{path}/Material"
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((0.85, 0.85, 0.8))  # Off-white
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(cube_prim).Bind(material)

# Get stage
from omni.isaac.core import World
world = World()
stage = world.stage

# Create walls (10m x 4m height, 0.2m thick)
create_wall(stage, "/World/wall_north", [5.0, 0.1, 2.0], [0, 5, 2], [0, 0, 0])
create_wall(stage, "/World/wall_south", [5.0, 0.1, 2.0], [0, -5, 2], [0, 0, 0])
create_wall(stage, "/World/wall_east", [0.1, 5.0, 2.0], [5, 0, 2], [0, 0, 0])
create_wall(stage, "/World/wall_west", [0.1, 5.0, 2.0], [-5, 0, 2], [0, 0, 0])

# Create ceiling
create_wall(stage, "/World/ceiling", [5.0, 5.0, 0.1], [0, 0, 4], [0, 0, 0])

print("Room created!")
```

**Run the script**. You now have an enclosed room.

### Step 3: Lighting Fundamentals

**Default lighting problem**: Isaac Sim's default lighting is a single distant light (like outdoor sun). Indoor environments need multiple light sources.

**Lighting types**:

1. **HDRI Dome Light**: 360° environment image providing realistic ambient lighting
2. **Rectangle Light**: Area light (like window or ceiling panel)
3. **Sphere Light**: Point light (like light bulb)
4. **Distant Light**: Directional light (like sun)

### Discovering Lighting Patterns

**Your initial attempt**:
```
I created an indoor room in Isaac Sim. With the default distant light, the scene looks flat (hard shadows, no ambient light). What lighting setup would simulate a warehouse with overhead fluorescents and some window light?
```

**AI's suggestion**:
```
For indoor warehouse lighting:

1. HDRI dome light (low intensity): Provides ambient/skylight through windows
   - Intensity: 500-1000
   - Texture: Use warehouse or industrial HDRI

2. Rectangle lights (overhead): Simulate fluorescent ceiling panels
   - Intensity: 3000-5000 each
   - Position: Grid pattern on ceiling (every 3-4 meters)
   - Size: 1m x 0.3m (fluorescent panel dimensions)
   - Color temperature: 4000-5000K (cool white)

3. Optional: Rectangle light for window (directional)
   - Intensity: 2000-3000
   - Position: Side wall, angled down
   - Simulates indirect sunlight through warehouse windows

This creates varied lighting (not uniform) which improves model robustness.
```

**What emerged**: AI suggested a specific lighting pattern (HDRI + grid of overheads + window) with grid pattern and intensity values optimized for realistic rendering.

### Step 4: Implementing HDRI Lighting

**Add dome light**:
1. **Create → Light → Dome Light**
2. In Property panel:
   - **Intensity**: 800
   - **Texture**: Browse to `localhost/NVIDIA/Assets/Skies/Indoor/warehouse_01.hdr`
   - **Visible**: Unchecked (don't render dome in background)

**Observe scene**: Lighting immediately looks more realistic—soft ambient light, no harsh shadows.

### Step 5: Adding Ceiling Panel Lights

**Create overhead lights** (manual for learning; Lesson 5 will automate this):
```python
from pxr import UsdLux

# Create rectangle light
def create_rect_light(stage, path, position, intensity=3500, width=1.0, height=0.3):
    light = UsdLux.RectLight.Define(stage, path)
    light.CreateIntensityAttr(intensity)
    light.CreateWidthAttr(width)
    light.CreateHeightAttr(height)
    light.CreateColorAttr((1.0, 0.95, 0.9))  # Warm white (4000K)

    # Position
    xform = UsdGeom.Xformable(light)
    xform.AddTranslateOp().Set(position)

    # Point downward
    xform.AddRotateXYZOp().Set((90, 0, 0))  # Rotate to point down

    return light

# Create grid of 4 ceiling lights
positions = [
    [-2.5, -2.5, 3.8],
    [-2.5, 2.5, 3.8],
    [2.5, -2.5, 3.8],
    [2.5, 2.5, 3.8]
]

for i, pos in enumerate(positions):
    create_rect_light(world.stage, f"/World/ceiling_light_{i}", pos)

print("Ceiling lights created!")
```

**Click Play**. Observe:
- Soft shadows under objects (not hard edges)
- Varied light intensity across room (realistic)
- Humanoid has subtle shadow variations

## RTX Render Settings

**Your scene is lit, but performance is slow (10-15 FPS). How to optimize?**

### Refining Render Quality

**Your question**:
```
My scene has HDRI dome + 4 ceiling lights. Rendering at 12 FPS (RTX mode). For synthetic data generation, I need 30+ FPS. What settings should I reduce without losing training value?
```

**AI's guidance**:
```
For training data generation (vs final visualization), reduce these settings:

1. Ray tracing samples:
   - High quality (offline): 256-512 samples
   - Real-time (data generation): 32-64 samples
   - You're generating thousands of images; slight noise is acceptable

2. Max bounces:
   - High quality: 8-12 bounces
   - Real-time: 2-4 bounces
   - Indirect illumination is still present, just less accurate

3. Denoising:
   - Enable denoiser to compensate for lower samples
   - NVIDIA OptiX denoiser is GPU-accelerated (minimal cost)

Try: 64 samples, 3 bounces, denoiser enabled. Should give 30-40 FPS on RTX 4070.
```

**Apply settings**:
1. **Render Settings** (lightbulb icon)
2. **RTX - Interactive (Path Tracing)**
   - **Samples per Pixel**: 64 (was 256)
   - **Max Bounces**: 3 (was 8)
   - **Denoiser**: Enabled

**Result**: FPS increases to 35-40. Visual quality still photorealistic (slight noise, but acceptable for training).

**What emerged**: Through dialogue, you and AI converged on settings that balance quality vs performance for your specific use case (dataset generation). AI's initial suggestion was good; you might further tune based on GPU.

## Material Coherence Challenge

**Observed issue**: You randomly assign materials—concrete floor, metal walls, wood ceiling. Scene looks incoherent.

### Teaching Material Constraints

**Your constraint**:
```
I'm randomizing materials for training robustness, but randomly assigning marble floors with rusted metal ceilings creates unrealistic scenes. What's a systematic approach to material assignment that preserves realism?
```

**AI's adapted approach**:
```
Define material categories by environment type:

**Warehouse environment**:
- Floor: Concrete (polished or aged), Epoxy coating
- Walls: Painted drywall, Concrete block, Corrugated metal
- Ceiling: Exposed metal beams, Acoustic tiles
- Objects: Cardboard boxes, Plastic bins, Metal shelves

**Office environment**:
- Floor: Carpet (various colors), Hardwood, Tile
- Walls: Painted drywall, Wood paneling, Glass partitions
- Ceiling: Drop ceiling tiles, Plaster
- Objects: Wood furniture, Fabric chairs, Glass tables

**Rule**: Select materials within one category. Don't mix warehouse + office in same scene.

This preserves contextual coherence (materials that co-occur in reality).
```

**Implementation**: You create material groups (warehouse_floor, warehouse_walls) and sample within groups. This **correlated randomization** pattern (Lesson 5 will formalize this) ensures contextual coherence.

## Comparison: RTX vs Rasterization

**Render the same scene in both modes**:

**RTX (Path Tracing)**:
- Soft shadows (penumbra from area lights)
- Accurate reflections (on floor if polished)
- Global illumination (light bouncing off walls brightens shadows)
- 35 FPS (on RTX 4070, with settings above)

**Rasterization**:
- Hard shadow edges (no soft shadows from area lights)
- Screen-space reflections only (no true ray-traced reflections)
- No global illumination (shadows are pure black)
- 90 FPS

**Socratic question**: *You're training a humanoid to navigate a warehouse at night with single overhead light. Model must learn to see in dim conditions (low light in shadows). Which render mode should you use for training data?*

<details>
<summary>Think before expanding</summary>

**RTX is essential** because:
- Global illumination means shadows aren't pure black (realistic low-light)
- Real warehouse has light bouncing off walls/floor (indirect illumination)
- Model trained on rasterized data (black shadows) would fail in real warehouse

**Rasterization limitation**: Cannot simulate subtle light in shadowed areas (model learns shadows = unobservable).
</details>

## Validation Checklist

- [ ] **HDRI dome light** provides ambient lighting (no pure black shadows)
- [ ] **Overhead lights** create varied intensity across scene (not uniform)
- [ ] **Materials use PBR**: Roughness and metallic set appropriately
- [ ] **Render quality**: 30+ FPS with photorealistic shadows
- [ ] **Material coherence**: Materials belong to same environment type
- [ ] **Shadow softness**: Shadows have soft edges (penumbra visible)

**Exercise**: Create two scenes:
1. **Warehouse**: Concrete floor, metal walls, industrial HDRI, overhead panel lights
2. **Office**: Carpet floor, painted walls, office HDRI, ceiling fixtures

Render both at 30+ FPS using optimized RTX settings.

## Try With AI

**Setup**: Open ChatGPT or Claude while working in Isaac Sim.

**Part 1: Initial Lighting Design**
Ask AI:
```
I'm creating a synthetic training environment for a service robot in a hotel lobby. The lobby has:
- Large windows (natural light)
- Chandelier (central ceiling)
- Wall sconces (ambient)
- Marble floor (reflective)

What Isaac Sim lighting setup (HDRI + light types + intensities) would simulate this realistically?
```

**Part 2: Performance Tuning**
After implementing AI's lighting, ask:
```
With the lobby scene above, I'm getting 18 FPS in RTX mode (need 30+ FPS). I have:
- HDRI dome (intensity 1000)
- 1 sphere light (chandelier, intensity 5000)
- 4 rectangle lights (wall sconces, intensity 1500 each)

My GPU is RTX 3070 (8GB VRAM). What should I reduce: light count, intensities, or render settings? Why?
```

Test AI's recommendation. Does FPS improve without losing realism?

**Part 3: Material Coherence**
Tell AI your specific environment:
```
For my hotel lobby scene, I want material variation for training robustness. I'm considering:
- Floor: Marble (polished, gray tones)
- Walls: Wood paneling OR painted drywall OR wallpaper
- Furniture: Leather sofas, glass coffee table, metal lamps

Which combinations preserve hotel lobby coherence? Which would create unrealistic scenes?
```

**Part 4: Final Convergence Check**
Compare your optimized scene to initial attempt:
- What lighting patterns did AI suggest?
- How did you refine based on your FPS constraints?
- What material coherence rules emerged from dialogue?
- Is the final scene both photorealistic AND performant? If not, what tradeoffs remain?

**Safety note**: High-intensity lights (&gt;10,000) can cause overexposure (washed-out images). Start with lower intensities and increase incrementally while checking exposure.
