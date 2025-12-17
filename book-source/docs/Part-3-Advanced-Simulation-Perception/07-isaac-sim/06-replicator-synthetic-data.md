# Lesson 6: Generating Synthetic Data with Replicator

Domain randomization (Lesson 5) creates scene diversity. Now you'll **scale** that diversity using Isaac Sim's Replicator API—generating thousands of annotated images automatically.

Replicator is Isaac Sim's synthetic data generation framework. It coordinates randomization, camera rendering, and annotation export in a single pipeline.

## Replicator Architecture

**Replicator workflow** (graph-based):
```
Randomizers → Scene → Cameras → Renderers → Writers → Annotations
     ↓          ↓         ↓           ↓           ↓          ↓
   Vary     Apply to   Capture    Generate     Export    Ground truth
   scene     objects   frames      images      formats    labels
```

**Key components**:
1. **Randomizers**: Functions that vary scene (textures, lighting, poses)
2. **Triggers**: When to randomize (every frame, every N frames, on event)
3. **Cameras**: Virtual cameras capturing scene from different viewpoints
4. **Writers**: Output formats (RGB, depth, bounding boxes, segmentation)

## Setting Up Replicator Pipeline

### Step 1: Import Replicator

```python
import omni.replicator.core as rep
from omni.isaac.core import World
import numpy as np

# Initialize Isaac Sim world
world = World()
world.reset()
```

### Step 2: Define Randomizers

**Consolidate Lesson 5 randomization**:
```python
def randomize_scene():
    """Full scene randomization from Lesson 5."""

    # Floor materials
    floor_materials = [
        "omniverse://localhost/NVIDIA/Materials/vMaterials_2/Ground/Concrete_Aged_01.mdl",
        "omniverse://localhost/NVIDIA/Materials/vMaterials_2/Ground/Concrete_Polished.mdl",
        "omniverse://localhost/NVIDIA/Materials/vMaterials_2/Ground/Epoxy_Gray.mdl"
    ]

    # Randomize floor
    floor = rep.get.prims(path_pattern="/World/floor")
    with floor:
        rep.randomizer.materials(floor_materials)

    # Randomize lighting
    lights = rep.get.prims(path_pattern="/World/ceiling_light_*")
    with lights:
        rep.randomizer.attribute(
            attribute="intensity",
            values=rep.distribution.uniform(3000, 5000)
        )

    # Randomize object poses
    objects = rep.get.prims(path_pattern="/World/box_*")
    with objects:
        rep.randomizer.scatter_2d(
            surface_prims=floor,
            check_for_collisions=True
        )
        rep.randomizer.rotation(rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360)))

# Register randomizer
rep.randomizer.register(randomize_scene)
```

### Step 3: Configure Cameras

**Multiple camera positions** (varied viewpoints):
```python
def create_cameras():
    """Create multiple cameras for diverse viewpoints."""

    camera_positions = [
        (3.0, 3.0, 2.5),   # Corner view
        (-3.0, 3.0, 2.5),  # Opposite corner
        (0.0, 4.0, 1.8),   # Side view (human height)
        (0.0, 0.0, 4.5)    # Top-down view
    ]

    cameras = []
    for i, pos in enumerate(camera_positions):
        cam = rep.create.camera(
            position=pos,
            look_at=(0, 0, 1)  # Look at center of room
        )
        cameras.append(cam)

    return cameras

cameras = create_cameras()
```

**Why multiple cameras?** Different viewpoints → model learns object appearance from all angles (not just one perspective).

### Step 4: Configure Writers (Annotations)

**Writer types**:
- **RGB**: Standard color images
- **Depth**: Distance to camera (for depth estimation)
- **Bounding box 2D**: Object detection labels
- **Semantic segmentation**: Per-pixel class labels
- **Instance segmentation**: Per-pixel object instance IDs

#### Configuring YOLO Format Writer

**Your requirement**:
```
I need bounding box annotations in YOLO format for object detection training. My classes are: humanoid, box, table. Output should be txt files with format: <class_id> <x_center> <y_center> <width> <height> (all normalized 0-1).
```

**AI's implementation guidance**:
```python
# Configure YOLO writer
output_dir = "/home/username/synthetic_data/warehouse_dataset"

# Define semantic classes
rep.settings.set_semantic_segmentation_classes([
    "humanoid",
    "box",
    "table"
])

# Create YOLO annotation writer
writer = rep.WriterRegistry.get("YOLOWriter")
writer.initialize(
    output_dir=output_dir,
    bbox_format="yolo",  # Normalized coordinates
    classes=["humanoid", "box", "table"]
)

# Attach writer to render product
render_product = rep.create.render_product(cameras[0], (1280, 720))
writer.attach([render_product])
```

**What emerged**: AI knew YOLO format requires normalized coordinates and class IDs. You specified your classes; AI adapted the configuration.

### Discovering Annotation Accuracy Issues

**Your first generation attempt** (1000 images):
```python
# Generate 1000 frames
with rep.trigger.on_frame(num_frames=1000):
    rep.randomizer.randomize_scene()
```

**Observed problem**: Manual spot-check reveals 12% of bounding boxes are incorrect (miss objects or include background).

**Your question**:
```
My YOLO bounding boxes have 12% error rate (missing objects, wrong boundaries). What causes annotation errors in Replicator? How do I debug this?
```

**AI's debugging guidance**:
```
Common Replicator annotation errors:

1. **Small/occluded objects**: If object is &lt;2% of image, bbox may not generate
   - Fix: Filter out tiny objects or use instance segmentation

2. **Semantic segmentation not applied**: Objects need semantic class labels
   - Fix: Ensure all objects have semantic.class attribute set

3. **Overlapping objects**: Z-fighting causes segmentation ambiguity
   - Fix: Add small separation between objects (0.01m minimum)

4. **Transparent materials**: Glass/plastic with alpha &lt;1.0 confuses segmentation
   - Fix: Set alpha=1.0 for all objects during annotation (can randomize alpha separately)

Debug workflow:
1. Enable semantic segmentation visualization (see pixel colors)
2. Check if problem objects appear in segmentation
3. If missing → semantic class not set
4. If present but bbox wrong → Writer configuration issue
```

**Your refinement**:
```python
# Add semantic classes to all objects
def assign_semantic_classes():
    # Humanoid
    humanoid_prims = rep.get.prims(path_pattern="/World/humanoid/*")
    with humanoid_prims:
        rep.modify.semantics([("class", "humanoid")])

    # Boxes
    box_prims = rep.get.prims(path_pattern="/World/box_*")
    with box_prims:
        rep.modify.semantics([("class", "box")])

    # Tables
    table_prims = rep.get.prims(path_pattern="/World/table_*")
    with table_prims:
        rep.modify.semantics([("class", "table")])

assign_semantic_classes()
```

**Retest**: Error rate drops to 3% (within acceptable range).

## Batch Generation and Performance

**Challenge**: Generating 10,000 images takes time. How to optimize?

### Tuning Batch Performance

**Your performance question**:
```
Generating 10,000 images (1280x720, RTX mode) takes 8 hours on my RTX 3070. Is this normal? Can I optimize without losing quality?
```

**AI's optimization strategy**:
```
**Performance optimization hierarchy** (try in order):

1. **Batch rendering** (biggest gain):
   - Render multiple cameras simultaneously
   - Isaac Sim can render 4-8 cameras in parallel (same frame, different views)
   - Speedup: 3-4x

2. **Reduce render quality** (moderate gain):
   - Samples: 64 → 32 (minor quality loss)
   - Resolution: 1280x720 → 640x480 (if acceptable for training)
   - Speedup: 1.5-2x

3. **GPU memory management**:
   - Unload unused textures between randomization
   - Use texture streaming (load on-demand)
   - Speedup: 1.2-1.5x (prevents OOM, allows larger batches)

Try batch rendering first (least quality impact, biggest gain).
```

**Implementation** (batch rendering):
```python
# Render from all 4 cameras simultaneously
render_products = []
for i, cam in enumerate(cameras):
    rp = rep.create.render_product(cam, (640, 480))  # Reduced resolution
    render_products.append(rp)

# Attach all render products to writer
writer.attach(render_products)

# Generate with all cameras
with rep.trigger.on_frame(num_frames=2500):  # 2500 frames × 4 cameras = 10,000 images
    rep.randomizer.randomize_scene()
```

**Result**: Generation time drops to 3 hours (2.7x speedup from batch rendering + resolution reduction).

**What emerged**: Through iterating on constraints (time budget) and exploring options (batch vs quality vs resolution), you and AI converged on a configuration optimized for your hardware.

## Dataset Organization

**Generated files need structure** for training pipelines.

### Standard Dataset Format

**YOLO dataset structure**:
```
warehouse_dataset/
├── images/
│   ├── train/
│   │   ├── img_0000.png
│   │   ├── img_0001.png
│   │   └── ...
│   └── val/
│       ├── img_8000.png
│       └── ...
├── labels/
│   ├── train/
│   │   ├── img_0000.txt
│   │   ├── img_0001.txt
│   │   └── ...
│   └── val/
│       └── ...
└── data.yaml
```

**data.yaml** (YOLO config):
```yaml
train: /path/to/warehouse_dataset/images/train
val: /path/to/warehouse_dataset/images/val

nc: 3  # Number of classes
names: ['humanoid', 'box', 'table']
```

**Replicator output** needs post-processing to match this format:

```python
import os
import shutil

def organize_dataset(replicator_output_dir, organized_output_dir, train_split=0.8):
    """Organize Replicator output into YOLO format."""

    # Create directory structure
    os.makedirs(f"{organized_output_dir}/images/train", exist_ok=True)
    os.makedirs(f"{organized_output_dir}/images/val", exist_ok=True)
    os.makedirs(f"{organized_output_dir}/labels/train", exist_ok=True)
    os.makedirs(f"{organized_output_dir}/labels/val", exist_ok=True)

    # List all generated images
    images = sorted([f for f in os.listdir(replicator_output_dir) if f.endswith('.png')])
    num_train = int(len(images) * train_split)

    # Split and copy
    for i, img_file in enumerate(images):
        label_file = img_file.replace('.png', '.txt')
        subset = "train" if i < num_train else "val"

        shutil.copy(
            f"{replicator_output_dir}/{img_file}",
            f"{organized_output_dir}/images/{subset}/{img_file}"
        )
        shutil.copy(
            f"{replicator_output_dir}/{label_file}",
            f"{organized_output_dir}/labels/{subset}/{label_file}"
        )

    print(f"Organized {len(images)} images: {num_train} train, {len(images) - num_train} val")

# Run organization
organize_dataset(
    replicator_output_dir="/home/username/isaac_output",
    organized_output_dir="/home/username/warehouse_dataset"
)
```

## Validation: Dataset Quality

**Before training models, validate dataset**:

### Automated Validation Checks

```python
def validate_dataset(dataset_dir):
    """Run quality checks on generated dataset."""

    errors = []

    # Check 1: Image-label pairs exist
    img_dir = f"{dataset_dir}/images/train"
    label_dir = f"{dataset_dir}/labels/train"

    images = set([f.replace('.png', '') for f in os.listdir(img_dir)])
    labels = set([f.replace('.txt', '') for f in os.listdir(label_dir)])

    missing_labels = images - labels
    if missing_labels:
        errors.append(f"{len(missing_labels)} images missing labels")

    # Check 2: Bounding box validity
    for label_file in os.listdir(label_dir):
        with open(f"{label_dir}/{label_file}") as f:
            for line_num, line in enumerate(f):
                parts = line.strip().split()
                if len(parts) != 5:
                    errors.append(f"{label_file}:{line_num} - Invalid format")
                    continue

                class_id, x, y, w, h = map(float, parts)

                # Check normalized coordinates
                if not (0 <= x <= 1 and 0 <= y <= 1 and 0 < w <= 1 and 0 < h <= 1):
                    errors.append(f"{label_file}:{line_num} - Out of range coordinates")

    # Check 3: Class distribution
    class_counts = {0: 0, 1: 0, 2: 0}  # humanoid, box, table
    for label_file in os.listdir(label_dir):
        with open(f"{label_dir}/{label_file}") as f:
            for line in f:
                class_id = int(line.split()[0])
                class_counts[class_id] += 1

    print(f"Class distribution: {class_counts}")
    if min(class_counts.values()) < 100:
        errors.append("At least one class has &lt;100 instances (imbalanced dataset)")

    # Report
    if errors:
        print(f"Validation FAILED: {len(errors)} errors")
        for err in errors[:10]:  # Show first 10
            print(f"  - {err}")
    else:
        print("Validation PASSED!")

    return len(errors) == 0

# Run validation
validate_dataset("/home/username/warehouse_dataset")
```

### Manual Spot-Check

**Visualize random samples**:
```python
import cv2
import random

def visualize_sample(dataset_dir, num_samples=10):
    """Display images with bounding boxes overlaid."""

    img_dir = f"{dataset_dir}/images/train"
    label_dir = f"{dataset_dir}/labels/train"

    image_files = os.listdir(img_dir)
    samples = random.sample(image_files, num_samples)

    for img_file in samples:
        img_path = f"{img_dir}/{img_file}"
        label_path = f"{label_dir}/{img_file.replace('.png', '.txt')}"

        # Load image
        img = cv2.imread(img_path)
        h, w = img.shape[:2]

        # Load bounding boxes
        with open(label_path) as f:
            for line in f:
                class_id, x, y, bw, bh = map(float, line.split())

                # Convert normalized to pixel coordinates
                x1 = int((x - bw/2) * w)
                y1 = int((y - bh/2) * h)
                x2 = int((x + bw/2) * w)
                y2 = int((y + bh/2) * h)

                # Draw rectangle
                color = [(255,0,0), (0,255,0), (0,0,255)][int(class_id)]  # BGR
                cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

        # Display (or save)
        cv2.imshow(f"Sample: {img_file}", img)
        cv2.waitKey(0)

    cv2.destroyAllWindows()

# Visualize 10 random samples
visualize_sample("/home/username/warehouse_dataset")
```

**Manual checklist** (review displayed samples):
- [ ] Bounding boxes tightly fit objects (not too large/small)
- [ ] All visible objects have boxes (no missing annotations)
- [ ] Occluded objects handled correctly (box around visible portion)
- [ ] Scene diversity evident (varied materials, lighting, poses)

## Exercise: Generate 1,000-Image Dataset

**Task**: Use Replicator to generate a complete dataset.

**Requirements**:
- 1,000 images (800 train, 200 validation)
- 3 classes: humanoid, box, table
- YOLO format annotations
- Multiple camera viewpoints (at least 3)
- Domain randomization (texture, lighting, pose)

**Success criteria**:
- [ ] Dataset passes automated validation (no errors)
- [ ] Manual spot-check shows high-quality annotations (&gt;95% accurate)
- [ ] Class distribution balanced (each class: 25-40% of total boxes)
- [ ] Generation completes in &lt;2 hours on your hardware

## Try With AI

**Setup**: Open ChatGPT or Claude.

**Part 1: Writer Configuration**
Ask AI:
```
I need to export annotations in COCO format (not YOLO). My classes are: person, robot_arm, conveyor_belt. Show me the Replicator writer configuration for COCO JSON format with bounding boxes and segmentation masks.
```

Implement and compare to YOLO format from this lesson.

**Part 2: Batch Optimization**
After implementing, ask:
```
My Replicator pipeline generates:
- 4 cameras (640x480 each)
- 64 samples per pixel (RTX)
- Domain randomization (10 textures loaded per frame)

I'm getting 12 FPS on RTX 3070 (8GB VRAM). To generate 10,000 images in 3 hours, I need 25+ FPS. What should I reduce: camera count, resolution, samples, or texture variety?
```

Test AI's recommendation. Does FPS improve to target?

**Part 3: Constraint Teaching**
Tell AI your specific need:
```
My object detection model will run on a Jetson Nano (512x512 input). Should I:
1. Generate at 512x512 (faster generation)
2. Generate at 1280x720, downsample during training (more detail)

Consider sim-to-real transfer and model accuracy.
```

**Part 4: Final Convergence**
After generating your dataset:
- What annotations formats did you explore (YOLO, COCO)?
- How did you balance quality vs generation speed?
- What emerged from AI collaboration that you hadn't considered?
- Is your dataset production-ready? If not, what's missing?

**Safety note**: Large dataset generation (10,000+ images) can fill disk space quickly. Ensure 50GB+ free space before starting. Monitor disk usage during generation.
