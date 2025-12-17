# Lesson 10: Capstone - Synthetic Training Dataset Generation

You've mastered Isaac Sim fundamentals (Lessons 1-2), AI collaboration for scene building (Lessons 3-7), and intelligence design (Lessons 8-9). Now you'll orchestrate everything into a production-ready capstone: **Generate 10,000+ photorealistic annotated images for humanoid navigation training**.

This is specification-first development: write detailed spec.md BEFORE implementation, then compose accumulated skills to execute the spec efficiently.

## Capstone Project Overview

**Goal**: Create a complete synthetic training dataset for object detection models that will power your humanoid's perception system.

**Dataset requirements**:
- **Quantity**: 10,000+ RGB images with YOLO bounding box annotations
- **Quality**: Photorealistic rendering (RTX ray tracing)
- **Diversity**: 20+ material variations, 10+ lighting conditions, 50+ object poses
- **Performance**: ≥30 FPS generation rate
- **Validation**: &lt;5% annotation errors, &gt;95% scene plausibility

**Why this dataset matters**: Part 4 will use this dataset to train object detection models for autonomous navigation. Quality directly impacts robot performance.

## Phase 1: Specification Writing (FIRST)

**Critical**: Write complete specification BEFORE any implementation.

### Step 1: Create spec.md

**File**: `~/capstone_dataset/spec.md`

**Template**:
```markdown
# Synthetic Training Dataset Specification v1.0

**Project**: Humanoid Object Detection Training Data
**Author**: [Your name]
**Date**: 2025-12-17
**Status**: Draft → Review → Approved → Implementation

---

## 1. Intent

Generate synthetic training dataset for object detection model. Model will detect obstacles in indoor warehouse environments to enable humanoid autonomous navigation (Part 4, Chapter 9).

**Use case**: Humanoid navigates warehouse, detects boxes/pallets/forklifts, plans collision-free paths.

---

## 2. Constraints

### Technical Constraints
- **Hardware**: RTX 3070 (8GB VRAM), 16GB RAM, Ubuntu 22.04
- **Software**: Isaac Sim 2023.1.1, ROS 2 Humble
- **Time budget**: Generation must complete in 8 hours (overnight run)

### Dataset Constraints
- **Format**: YOLO (normalized bounding boxes)
- **Resolution**: 640x480 (matches training pipeline input)
- **Split**: 80% train (8,000 images), 20% validation (2,000 images)
- **Classes**: 3 classes (humanoid, box, pallet)

### Quality Constraints
- **Annotation accuracy**: &gt;95% (spot-check 100 samples)
- **Scene plausibility**: &gt;95% (manual review)
- **Class balance**: Each class 25-40% of total detections

---

## 3. Success Criteria

- [ ] **Quantity**: 10,000+ images generated
- [ ] **Annotations**: All images have accurate YOLO labels (class_id x y w h)
- [ ] **Diversity**: Measured via distribution analysis
  - Materials: ≥20 unique variations (floor, walls, objects)
  - Lighting: 10+ configurations (intensity 3000-5000)
  - Poses: 50+ object positions per class
- [ ] **Performance**: Generation rate ≥30 FPS (8 hours for 10K images)
- [ ] **Validation**: &lt;5% annotation errors, &gt;95% plausibility
- [ ] **Deliverables**:
  - `images/train/` (8,000 PNG files)
  - `images/val/` (2,000 PNG files)
  - `labels/train/` (8,000 TXT files)
  - `labels/val/` (2,000 TXT files)
  - `data.yaml` (YOLO config)
  - `dataset_report.md` (quality metrics)

---

## 4. Non-Goals (Explicit Exclusions)

- ❌ **Depth maps**: Only RGB + bounding boxes (depth not needed for object detection)
- ❌ **Semantic segmentation**: Bounding boxes sufficient for navigation
- ❌ **Video sequences**: Static images only (tracking deferred to Part 4)
- ❌ **Outdoor scenes**: Warehouse indoors only
- ❌ **Dynamic obstacles**: Static scene snapshots (humanoid/objects stationary)

**Why excluded**: Scope management. These features can be added in future dataset versions if needed.

---

## 5. Component Composition (Intelligence Reuse)

### Skills to Apply

**From Lesson 8**: isaac-sim-performance skill
- Optimize render settings (samples, bounces, batch size)
- Profile bottlenecks
- Target 30+ FPS

**From Lesson 9**: isaac-sim-domain-randomization skill
- Correlated material randomization (floor-wall coherence)
- Lighting-material correlation
- Plausibility constraints
- Validation metrics

### Knowledge to Apply

**From Lesson 3**: URDF import
- Humanoid robot configured with stable joint drives

**From Lesson 4**: Photorealistic rendering
- RTX settings (64 samples, 3 bounces, denoiser enabled)
- PBR materials
- HDRI + ceiling lights

**From Lesson 5**: Domain randomization
- Texture variation (warehouse materials)
- Lighting variation (3500-5000 intensity)
- Pose randomization (scatter_2d with collision checking)

**From Lesson 6**: Replicator pipeline
- YOLO writer configuration
- Batch rendering (4 cameras)
- Semantic class assignment

**From Lesson 7**: ROS 2 bridge (optional, not used for dataset generation)

---

## 6. Implementation Plan

### Phase 1: Scene Setup (1 hour)
1. Create warehouse environment (10m x 10m room)
2. Import humanoid URDF with stable joint configuration
3. Add objects (10 boxes, 5 pallets)
4. Configure camera (4 viewpoints for batch rendering)

### Phase 2: Randomization Configuration (1 hour)
1. Apply domain-randomization skill (materials, lighting, poses)
2. Implement plausibility checks
3. Test with 100 sample generations

### Phase 3: Replicator Pipeline (30 minutes)
1. Configure YOLO writer (class labels, output directory)
2. Create batch render products (4 cameras @ 640x480)
3. Attach writers to render products

### Phase 4: Performance Optimization (1 hour)
1. Apply performance skill (profile bottleneck)
2. Tune render settings for 30+ FPS target
3. Verify GPU memory within limits (< 7GB VRAM usage)

### Phase 5: Generation (8 hours)
1. Run Replicator for 10,000 images
2. Monitor progress (check FPS, error logs)

### Phase 6: Validation (2 hours)
1. Automated checks (bounding box validity, class distribution)
2. Manual spot-check (100 random samples)
3. Generate dataset_report.md with metrics

**Total estimated time**: 13.5 hours (including validation)

---

## 7. Risk Analysis

| Risk | Impact | Mitigation |
|------|--------|------------|
| GPU OOM during generation | High (crash loses progress) | Monitor VRAM, reduce batch size if &gt;90% usage |
| Annotation errors &gt;5% | High (unusable for training) | Test with 1,000 images first, validate before scaling |
| Generation time &gt;8 hours | Medium (delays project) | Optimize early (Phase 4), accept lower resolution if needed |
| Implausible scenes &gt;5% | Medium (harms model quality) | Apply plausibility checks, manual review of 100 samples |

---

## 8. Validation Plan

### Automated Validation
```python
def validate_dataset(dataset_dir):
    errors = []

    # Check 1: Image-label pairs
    # Check 2: Bounding box format validity
    # Check 3: Class distribution balance

    return errors
```

### Manual Validation
- Spot-check 100 random images
- Verify bounding boxes fit objects
- Check scene plausibility (no grass floors, etc.)

### Success Metrics
- Annotation accuracy: &gt;95%
- Scene plausibility: &gt;95%
- Class balance: Each class 25-40% of total boxes

---

## 9. Acceptance Criteria

**This specification is accepted when**:
- [ ] All sections complete (no TODOs)
- [ ] Success criteria measurable (not vague)
- [ ] Non-goals explicitly stated (scope boundaries clear)
- [ ] Skills/knowledge reuse identified (not reinventing)
- [ ] Risk mitigation strategies defined

**Ready for implementation after**:
- Specification reviewed by peer (or AI for feedback)
- All acceptance criteria met
```

**Save spec.md**. DO NOT start implementation yet.

### Step 2: Review Specification with AI

**Before implementation, get feedback on spec**:

**Your request**:
```
Review this dataset generation specification. What's missing? What risks did I not consider? Are success criteria measurable?

[Paste complete spec.md]
```

**AI's feedback** (example):
```
**Strengths**:
- Clear success criteria (measurable quantities)
- Explicit non-goals (scope well-defined)
- Risk mitigation strategies present

**Gaps identified**:

1. **Lighting validation**: Success criteria mention "10+ lighting configurations" but don't specify how to measure this
   - Add: "Lighting intensity distribution should span 3000-5000 range (measure std dev)"

2. **Failure recovery**: What if generation crashes at 7,000 images?
   - Add to risks: Checkpoint saving every 1,000 images

3. **Hardware degradation**: 8-hour GPU run could cause thermal throttling
   - Add to risks: Monitor GPU temperature, pause if &gt;85°C

4. **Data versioning**: How to track dataset versions (v1.0, v1.1)?
   - Add to deliverables: VERSION.txt with dataset metadata

**Recommendation**: Address gaps, then approve spec for implementation.
```

**Refine spec** based on feedback, then mark status as **Approved**.

## Phase 2: Implementation (Orchestrating Skills)

**Now that spec is approved, implement using accumulated intelligence.**

### Step 1: Scene Setup Script

```python
# capstone_scene_setup.py
import omni.replicator.core as rep
from omni.isaac.core import World
import numpy as np

# Initialize world
world = World()
world.reset()

# Create warehouse room (from Lesson 4)
def create_warehouse_room():
    # Floor
    floor = rep.create.plane(
        scale=(10, 10, 1),
        position=(0, 0, 0)
    )

    # Walls (simplified for capstone)
    # ... (reuse Lesson 4 code)

    return floor

# Import humanoid (from Lesson 3)
def import_humanoid():
    # Use URDF importer with stable joint configuration
    # ... (reuse Lesson 3 configuration)
    pass

# Add objects
def create_objects():
    boxes = []
    for i in range(10):
        box = rep.create.cube(
            scale=(0.5, 0.5, 0.5),
            position=(0, 0, 0.25),  # Will be randomized
            semantics=[("class", "box")]
        )
        boxes.append(box)

    pallets = []
    for i in range(5):
        pallet = rep.create.cube(
            scale=(1.2, 0.8, 0.15),
            position=(0, 0, 0.075),
            semantics=[("class", "pallet")]
        )
        pallets.append(pallet)

    return boxes, pallets

# Setup scene
floor = create_warehouse_room()
humanoid = import_humanoid()
boxes, pallets = create_objects()

print("Scene setup complete!")
```

### Step 2: Apply Domain Randomization Skill

**Invoke Lesson 9 skill**:
```python
# capstone_randomization.py (using skill principles)

def randomize_scene_with_skill():
    """Apply isaac-sim-domain-randomization skill."""

    # Question 1: What variations increase robustness?
    # Answer: Texture, lighting, pose (for warehouse object detection)

    # Question 2: What correlations preserve realism?
    # Answer: Floor-wall materials, lighting-reflectivity

    # Principle 1: Systematic variation
    environment_type = "warehouse"
    floor_material = np.random.choice([
        "concrete_aged", "concrete_polished", "epoxy_gray"
    ])

    # Principle 5: Correlation management
    wall_materials = {
        "concrete_aged": ["brick_red", "metal_corrugated"],
        "concrete_polished": ["painted_white", "metal_brushed"],
        "epoxy_gray": ["painted_blue", "concrete_block"]
    }
    wall_material = np.random.choice(wall_materials[floor_material])

    # Apply materials
    rep.randomizer.materials(
        floor_prim,
        materials=[get_material_path(floor_material)]
    )

    # Principle 2: Plausibility constraint
    base_lighting = np.random.uniform(3500, 5000)  # Warehouse range
    if floor_material == "concrete_aged":  # Dark floor
        base_lighting *= 1.2  # Brighter lights needed

    # Randomize lighting
    for light in ceiling_lights:
        light.intensity = base_lighting * np.random.uniform(0.85, 1.15)

    # Randomize poses (collision-aware)
    rep.randomizer.scatter_2d(
        objects=boxes + pallets,
        surface=floor_prim,
        check_for_collisions=True
    )

# Register randomizer
rep.randomizer.register(randomize_scene_with_skill)
```

### Step 3: Apply Performance Skill

**Optimize based on Lesson 8 skill**:
```python
# capstone_performance.py (using skill principles)

# Principle 1: Profile first
import subprocess
result = subprocess.run(["nvidia-smi"], capture_output=True)
print(result.stdout.decode())

# Question 1: What's the bottleneck?
# (Assume profiling shows GPU compute-bound, VRAM 5GB/8GB)

# Principle 4: Batch size = f(VRAM)
# RTX 3070 (8GB) with 5GB used → Can add 1-2 more cameras
num_cameras = 4  # Batch rendering

# Question 2: Which render settings impact quality vs speed?
render_settings = {
    "samples_per_pixel": 64,  # Reduced from 256 (acceptable noise)
    "max_bounces": 3,         # Reduced from 8 (indirect illumination still present)
    "resolution": (640, 480), # Matches training pipeline
    "denoiser_enabled": True  # Compensates for lower samples
}

# Apply settings
# ... (Isaac Sim render settings configuration)

print(f"Performance optimized for {num_cameras} cameras at 30+ FPS")
```

### Step 4: Generate Dataset

**Replicator pipeline** (composed from Lesson 6):
```python
# capstone_generation.py

import omni.replicator.core as rep

# Configure cameras (4 viewpoints for diversity)
cameras = []
camera_positions = [
    (4, 4, 2.5), (-4, 4, 2.5),
    (4, -4, 2.5), (-4, -4, 2.5)
]
for i, pos in enumerate(camera_positions):
    cam = rep.create.camera(position=pos, look_at=(0, 0, 1))
    cameras.append(cam)

# Configure YOLO writer
output_dir = "/home/username/capstone_dataset"
writer = rep.WriterRegistry.get("YOLOWriter")
writer.initialize(
    output_dir=output_dir,
    bbox_format="yolo",
    classes=["humanoid", "box", "pallet"]
)

# Attach all cameras (batch rendering)
render_products = []
for cam in cameras:
    rp = rep.create.render_product(cam, render_settings["resolution"])
    render_products.append(rp)

writer.attach(render_products)

# Generate 10,000 images (2,500 frames × 4 cameras)
print("Starting generation... (estimated 8 hours)")
with rep.trigger.on_frame(num_frames=2500):
    randomize_scene_with_skill()

print("Generation complete!")
```

### Step 5: Validation

**Run automated validation** (from spec.md):
```python
# capstone_validation.py

def validate_capstone_dataset(dataset_dir):
    """Validate generated dataset against spec criteria."""

    errors = []

    # Success Criterion 1: Quantity
    img_count = len(os.listdir(f"{dataset_dir}/images/train"))
    if img_count < 8000:
        errors.append(f"Insufficient images: {img_count}/8000")

    # Success Criterion 2: Annotations
    # (Check format, normalized coordinates, etc.)
    # ... (reuse validation from Lesson 6)

    # Success Criterion 3: Diversity
    # Measure material distribution
    material_distribution = analyze_material_diversity(dataset_dir)
    if len(material_distribution) < 20:
        errors.append(f"Insufficient material diversity: {len(material_distribution)}/20")

    # Success Criterion 4: Performance
    generation_time = read_log_file("generation.log")
    fps = 10000 / (generation_time / 4)  # 4 cameras
    if fps < 30:
        errors.append(f"Performance below target: {fps}/30 FPS")

    # Success Criterion 5: Validation
    annotation_accuracy = spot_check_annotations(dataset_dir, num_samples=100)
    if annotation_accuracy < 0.95:
        errors.append(f"Annotation accuracy: {annotation_accuracy:.2%}/95%")

    # Report
    if errors:
        print(f"Validation FAILED: {len(errors)} criteria not met")
        for err in errors:
            print(f"  ✗ {err}")
        return False
    else:
        print("Validation PASSED: All success criteria met!")
        return True

# Run validation
if validate_capstone_dataset("/home/username/capstone_dataset"):
    print("Dataset ready for training (Part 4)!")
else:
    print("Dataset needs refinement. Review errors above.")
```

## Deliverables Checklist

- [ ] **spec.md**: Complete specification (all sections filled)
- [ ] **capstone_scene_setup.py**: Scene creation script
- [ ] **capstone_randomization.py**: Domain randomization (skill-guided)
- [ ] **capstone_performance.py**: Performance optimization (skill-guided)
- [ ] **capstone_generation.py**: Replicator pipeline
- [ ] **capstone_validation.py**: Automated validation
- [ ] **Dataset**: 10,000+ images in YOLO format
- [ ] **dataset_report.md**: Quality metrics, validation results

## Try With AI

**Setup**: You've written spec.md, now collaborate with AI for implementation.

**Part 1: Specification Review**
```
Review my capstone dataset specification. Check for:
1. Measurable success criteria (not vague)
2. Missing risk mitigations
3. Skills composition (am I reusing Lessons 8-9?)

[Paste spec.md]
```

Refine spec based on feedback.

**Part 2: Implementation Orchestration**
```
I have these reusable components:
- isaac-sim-performance skill (Lesson 8)
- isaac-sim-domain-randomization skill (Lesson 9)
- URDF import configuration (Lesson 3)
- Replicator pipeline template (Lesson 6)

Generate Python implementation plan that composes these. My spec requirements:
[Paste relevant sections from spec.md]
```

**Part 3: Debugging Failed Validation**
```
My capstone validation failed on:
- Annotation accuracy: 92% (need 95%)
- Material diversity: 18 variations (need 20+)
- Performance: 25 FPS (need 30+)

Apply my performance and domain-randomization skills to debug. Which principle from each skill addresses these failures?
```

**Part 4: Final Convergence**
After completing capstone:
- Did specification-first approach reduce implementation errors?
- How much time did skills reuse save vs coding from scratch?
- What emerged from implementation that you didn't specify?
- Is your dataset production-ready for Part 4 (Chapter 10 training)?

**Safety note**: Long generation runs (8+ hours) can fail. Implement checkpoint saving every 1,000 images. If crash occurs, resume from last checkpoint instead of restarting from zero.
