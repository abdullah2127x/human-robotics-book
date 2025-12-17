# Lesson 5: Domain Randomization Fundamentals

Photorealistic rendering (Lesson 4) makes individual scenes look real. But perception models need **diversity**—thousands of varied scenes to learn robust patterns that generalize to unseen environments.

**Domain randomization** systematically varies texture, lighting, and object poses to create this diversity. Done correctly, it produces models that handle real-world variation (different warehouses, times of day, object arrangements).

Done incorrectly, it creates implausible scenes that harm training.

## Why Domain Randomization Works

**The insight**: Models trained on varied synthetic data often outperform models trained on limited real data.

**Example**:
- **Real data only**: 1,000 images from one warehouse → Model fails in different warehouse
- **Synthetic with randomization**: 10,000 images with varied textures, lighting, poses → Model generalizes to new warehouses

**Why**: Randomization forces model to learn invariant features (object shape, spatial relationships) rather than memorizing specific textures or lighting conditions.

## Types of Randomization

### 1. Texture Randomization

**What varies**: Surface materials (floor, walls, objects)

**Example variations**:
- Floor: Concrete (polished) → Concrete (aged) → Epoxy (gray) → Epoxy (blue)
- Walls: White paint → Beige paint → Exposed brick → Corrugated metal
- Boxes: Brown cardboard → White cardboard → Plastic bins (blue) → Plastic bins (red)

**Implementation** (Isaac Sim Randomization API):
```python
import omni.replicator.core as rep

# Define texture variations for floor
floor_materials = [
    "omniverse://localhost/NVIDIA/Materials/vMaterials_2/Ground/Concrete_Aged_01.mdl",
    "omniverse://localhost/NVIDIA/Materials/vMaterials_2/Ground/Concrete_Polished.mdl",
    "omniverse://localhost/NVIDIA/Materials/vMaterials_2/Ground/Epoxy_Gray.mdl"
]

# Randomize floor material
with rep.trigger.on_frame():
    rep.randomizer.materials(
        materials=rep.get.prims(path_pattern="/World/floor"),
        choices=floor_materials
    )

print("Texture randomization configured!")
```

### 2. Lighting Randomization

**What varies**: Light intensity, color temperature, position

**Example variations**:
- Overhead light intensity: 3000 → 5000 (simulates dimmer switch, aging bulbs)
- Color temperature: 3500K (warm) → 5500K (cool)
- HDRI rotation: 0° → 360° (simulates time of day via window direction)

**Implementation**:
```python
import numpy as np
from pxr import UsdLux

# Randomize ceiling light intensities
def randomize_lighting():
    for i in range(4):  # 4 ceiling lights from Lesson 4
        light_path = f"/World/ceiling_light_{i}"
        light = UsdLux.RectLight.Get(world.stage, light_path)

        # Random intensity (3000-5000 range)
        intensity = np.random.uniform(3000, 5000)
        light.GetIntensityAttr().Set(intensity)

        # Random color temperature (3500-5000K)
        temp = np.random.uniform(3500, 5000)
        # Convert Kelvin to RGB (simplified)
        if temp < 4000:
            color = (1.0, 0.9, 0.8)  # Warm
        else:
            color = (1.0, 0.95, 0.95)  # Cool
        light.GetColorAttr().Set(color)

# Randomize HDRI rotation
with rep.trigger.on_frame():
    rep.randomizer.rotation(
        prims=rep.get.prims(path_pattern="/World/dome_light"),
        rotation=(0, np.random.uniform(0, 360), 0)  # Y-axis rotation
    )
```

### 3. Pose Randomization

**What varies**: Object positions, orientations

**Example variations**:
- Box position: Varies within 5m x 5m region on floor
- Box orientation: Random rotation around vertical axis (0-360°)
- Box presence: Sometimes boxes present, sometimes absent (occlusion variation)

**Implementation**:
```python
# Randomize object poses
def randomize_object_poses(num_objects=10):
    for i in range(num_objects):
        obj_path = f"/World/box_{i}"

        # Random position within room bounds
        x = np.random.uniform(-4, 4)
        y = np.random.uniform(-4, 4)
        z = 0.5  # On floor (box half-height)

        # Random rotation around Z-axis
        rotation_z = np.random.uniform(0, 360)

        rep.modify.pose(
            prim=rep.get.prims(path_pattern=obj_path),
            position=(x, y, z),
            rotation=(0, 0, rotation_z)
        )
```

## The Correlation Challenge

**Problem**: Random Assignment can create unrealistic scenes.

### Discovering Correlated Randomization

**Your initial implementation**:
```python
# Naive randomization (WRONG)
floor_material = random.choice(["concrete", "marble", "grass", "sand"])
wall_material = random.choice(["brick", "metal", "fabric", "glass"])
lighting_intensity = random.uniform(100, 10000)  # Any intensity
```

**Observed outcome**: Warehouse with grass floor, fabric walls, and extremely dim lighting (intensity 150). Completely unrealistic.

**Your question**:
```
I'm randomizing materials and lighting independently. This creates implausible scenes (grass floors indoors, extremely dim warehouses). How do I ensure randomization stays realistic?
```

**AI's guidance**:
```
Use correlated randomization: Variables that co-occur in reality should be sampled together.

**Correlation examples**:
1. Material coherence: Warehouse environments have industrial materials together
   - If floor=concrete, walls should be brick/metal/painted (not grass/fabric)

2. Lighting-material correlation: Dark materials require brighter lighting
   - If floor=dark_concrete, ceiling_lights need higher intensity
   - If floor=white_tile, lower intensity suffices

3. Lighting consistency: All ceiling lights should have similar intensity
   - Not: light_1=5000, light_2=500 (unrealistic)
   - Instead: Sample base_intensity, vary each light by ±20%

**Implementation pattern**:
```python
# Environment-based material sampling
environment_type = random.choice(["warehouse", "office"])

if environment_type == "warehouse":
    floor = random.choice(["concrete_aged", "concrete_polished", "epoxy_gray"])
    walls = random.choice(["painted_drywall", "brick", "metal_corrugated"])
    base_light_intensity = random.uniform(3500, 5000)  # Industrial lighting
else:  # office
    floor = random.choice(["carpet_gray", "carpet_blue", "hardwood", "tile"])
    walls = random.choice(["painted_white", "painted_beige", "wood_panel"])
    base_light_intensity = random.uniform(2000, 3500)  # Office lighting

# Individual light variations (±15% from base)
for light in ceiling_lights:
    light.intensity = base_light_intensity * random.uniform(0.85, 1.15)
```
```

**What emerged**: Systematic correlated randomization—sample environment type first, then materials consistent with that type. This pattern emerged through dialogue about plausibility constraints.

### Implementing Correlated Randomization

**Full example** (warehouse environment):
```python
import omni.replicator.core as rep
import numpy as np

def randomize_warehouse_scene():
    """Randomize scene with correlated variables."""

    # Sample floor material
    floor_materials = {
        "concrete_aged": {
            "path": "omniverse://localhost/NVIDIA/Materials/vMaterials_2/Ground/Concrete_Aged_01.mdl",
            "roughness": 0.7,
            "reflectivity": 0.1
        },
        "concrete_polished": {
            "path": "omniverse://localhost/NVIDIA/Materials/vMaterials_2/Ground/Concrete_Polished.mdl",
            "roughness": 0.3,
            "reflectivity": 0.4
        },
        "epoxy_gray": {
            "path": "omniverse://localhost/NVIDIA/Materials/vMaterials_2/Ground/Epoxy_Gray.mdl",
            "roughness": 0.2,
            "reflectivity": 0.5
        }
    }

    floor_choice = np.random.choice(list(floor_materials.keys()))
    floor_mat = floor_materials[floor_choice]

    # Apply floor material
    rep.randomizer.materials(
        materials=rep.get.prims(path_pattern="/World/floor"),
        input_prims=[floor_mat["path"]]
    )

    # Correlate lighting with floor reflectivity
    # Dark/rough floors need brighter lights
    base_intensity = 3000 + (1.0 - floor_mat["reflectivity"]) * 2000  # 3000-5000 range

    # Randomize lights with ±15% variation
    for i in range(4):
        light_path = f"/World/ceiling_light_{i}"
        light = UsdLux.RectLight.Get(world.stage, light_path)
        intensity = base_intensity * np.random.uniform(0.85, 1.15)
        light.GetIntensityAttr().Set(intensity)

    # Randomize object poses
    for i in range(10):
        rep.modify.pose(
            prim=rep.get.prims(path_pattern=f"/World/box_{i}"),
            position=(
                np.random.uniform(-4, 4),
                np.random.uniform(-4, 4),
                0.5
            ),
            rotation=(0, 0, np.random.uniform(0, 360))
        )

# Run randomization
with rep.trigger.on_frame(num_frames=100):
    randomize_warehouse_scene()

print("Correlated randomization active for 100 frames!")
```

**Key insight**: Lighting intensity correlates with floor reflectivity. Bright polished floor needs less light; dark aged concrete needs more light to maintain similar overall brightness.

## Randomization Frequency Tradeoff

**Question**: How often to randomize?

### Finding the Right Frequency

**Your exploration**:
```
I'm generating video sequences (30 FPS) for training. Should I randomize every frame, every 10 frames, or once per video?
```

**AI's analysis**:
```
**Randomization frequency tradeoffs**:

1. **Every frame** (30 times/second):
   - Pro: Maximum diversity
   - Con: Creates motion blur artifacts (materials flickering)
   - Con: Model may learn randomization artifacts instead of objects

2. **Every N frames** (e.g., every 10 frames = 3 times/second):
   - Pro: Temporal consistency (objects don't flicker)
   - Pro: Still high diversity
   - Con: Requires tuning N for your use case

3. **Once per episode/video**:
   - Pro: Fully consistent video
   - Con: Less diversity per unit time

**Recommendation for training data**:
- Static images: Randomize every frame (maximum diversity)
- Video sequences: Randomize every 10-30 frames (balance consistency vs diversity)
- Simulation episodes: Randomize once per episode (if training RL agents)

For your humanoid navigation training (static images), randomize every frame.
```

**Your refinement**:
```
What if I'm training on both static images (object detection) AND video (tracking)? Same randomization strategy?
```

**AI's adaptation**:
```
Split your dataset:
1. Static image dataset: Randomize every frame (for object detection)
2. Video dataset: Randomize every 15 frames (for tracking, need temporal coherence)

This gives your model diverse static images while also learning temporal consistency in videos.
```

**What emerged**: Through questioning, you and AI converged on a dataset split strategy that serves both use cases. This wasn't obvious from the initial recommendation.

## Validation: Randomization Quality

**How do you know randomization is "good"?**

### Qualitative Checks

**Generate 100 randomized frames**, then manually review:
```python
# Generate samples
import omni.kit.app
for i in range(100):
    randomize_warehouse_scene()
    world.step(render=True)
    omni.kit.app.get_app().update()

    # Save frame (Replicator will auto-save)
```

**Review checklist**:
- [ ] **Plausibility**: All scenes look like real warehouses (no grass floors indoors)
- [ ] **Diversity**: Materials vary noticeably across images (not 90% same concrete)
- [ ] **Lighting realism**: No extremely dim or overexposed images
- [ ] **Object placement**: Boxes don't overlap, all on floor (not floating)
- [ ] **Material coherence**: Floor, walls, objects belong to same environment type

**If checklist fails**: Review correlation rules, adjust sampling distributions.

### Quantitative Checks

**Measure variation**:
```python
import numpy as np

# Collect material samples
material_samples = []
lighting_samples = []

for i in range(1000):
    randomize_warehouse_scene()
    material_samples.append(get_floor_material())  # Custom getter
    lighting_samples.append(get_ceiling_light_intensity())

# Check distribution
print(f"Unique materials: {len(set(material_samples))} / 3 possible")
print(f"Lighting mean: {np.mean(lighting_samples):.2f}, std: {np.std(lighting_samples):.2f}")

# Expect:
# - All 3 materials appear (not skewed to one)
# - Lighting has reasonable std (not all samples identical)
```

**Healthy randomization**:
- **Material distribution**: ~33% each (if 3 materials equally likely)
- **Lighting std**: 400-800 range (significant variation, not tiny noise)

## Exercise: Implement Correlated Randomization

**Task**: Extend the warehouse randomization to include wall materials correlated with floor.

**Rules to implement**:
1. If floor is `concrete_aged` (rough, industrial):
   - Walls should be: `brick_red`, `metal_corrugated`, or `painted_warehouse_gray`
2. If floor is `concrete_polished` (smooth, modern warehouse):
   - Walls should be: `painted_white`, `painted_beige`, or `metal_brushed`
3. If floor is `epoxy_gray` (industrial coating):
   - Walls should be: `painted_warehouse_blue`, `metal_corrugated`, or `concrete_block`

**Starter code**:
```python
def randomize_correlated_scene():
    floor_wall_correlation = {
        "concrete_aged": ["brick_red", "metal_corrugated", "painted_gray"],
        "concrete_polished": ["painted_white", "painted_beige", "metal_brushed"],
        "epoxy_gray": ["painted_blue", "metal_corrugated", "concrete_block"]
    }

    # Your implementation here
    pass
```

**Success criteria**:
- [ ] Generate 100 images with your implementation
- [ ] Manually review: All floor-wall combinations are plausible
- [ ] No forbidden combinations (e.g., `concrete_aged` + `painted_white`)

## Try With AI

**Setup**: Open ChatGPT or Claude.

**Part 1: Discovering Correlations**
Ask AI:
```
I'm generating synthetic data for a hotel lobby. I want to randomize:
- Floor materials (marble, tile, carpet)
- Wall materials (wood paneling, painted drywall, wallpaper)
- Furniture (modern, traditional, minimalist styles)

What correlations should I preserve? Give me rules like "if floor=marble, then furniture should be..."
```

Implement AI's correlation rules.

**Part 2: Frequency Tuning**
After implementing, ask:
```
With the randomization above, I'm generating:
- 5,000 static images (object detection training)
- 100 video clips of 5 seconds each (tracking training)

What randomization frequency for each dataset? Why?
```

**Part 3: Constraint Teaching**
Tell AI your specific constraints:
```
I'm using an RTX 3070 (8GB VRAM). My randomization includes:
- 3 floor materials (each 2GB texture)
- 5 wall materials (each 1GB texture)
- 10 object materials (each 500MB texture)

If I randomize every frame, textures must be loaded/unloaded constantly. This drops FPS to 8. How should I structure randomization to keep FPS above 25?
```

Test AI's optimization strategy.

**Part 4: Convergence Reflection**
After implementing optimized randomization:
- What correlation rules did AI suggest?
- How did you refine based on your domain (hotel vs warehouse)?
- What frequency emerged from performance constraints?
- Did collaboration produce better randomization than you'd design alone? Why?

**Safety note**: When randomizing poses, ensure objects don't spawn inside walls or overlapping each other. Add collision checks or spawn regions to prevent invalid configurations.
