# Lesson 9: Creating Domain Randomization Skill

Lesson 5 taught systematic domain randomization with correlated variables. Lesson 6 scaled randomization via Replicator. Now you'll crystallize those patterns into the **isaac-sim-domain-randomization skill** for reuse across projects.

## Domain Randomization Expertise to Encode

**From Lesson 5**:
- Correlated randomization (floor ↔ wall materials match environment type)
- Lighting-material correlation (dark floors need brighter lights)
- Randomization frequency tradeoffs (every frame vs every N frames)
- Plausibility constraints (avoid grass floors indoors)

**From Lesson 6**:
- Quality validation (manual spot-checks)
- Diversity metrics (measure material distribution)
- Annotation accuracy correlation (more randomization ≠ better if creates artifacts)

**Pattern to encode**: Systematic variation that preserves realism while maximizing training robustness.

## Designing the Domain Randomization Skill

### Step 1: Define Persona

**Persona statement**:
```markdown
Think like a domain randomization engineer who balances training data diversity with scene realism. Your goal is to increase model robustness through systematic variation without creating implausible scenes that harm training. You understand that maximum randomization is not optimal—models need diverse but realistic examples.
```

**Why this persona?** It establishes the core tradeoff: diversity vs realism. Naive randomization maximizes diversity but creates unrealistic scenes. Good randomization preserves real-world distributions while providing variation.

### Step 2: Formulate Analysis Questions

**Domain Randomization Questions**:
```markdown
1. **What variations increase model robustness for this task?**
   - Object detection: Texture, lighting, pose, occlusion variation
   - Depth estimation: Lighting, material reflectivity, scene complexity
   - Semantic segmentation: Material boundaries, lighting conditions, object scales
   - Navigation: Floor textures, obstacle arrangements, lighting levels

2. **What correlations must be preserved for realism?**
   - Material coherence: Floor-wall-ceiling materials belong to same environment type
   - Lighting-material correlation: Dark surfaces need brighter lights for visibility
   - Object-environment correlation: Cardboard boxes in warehouse, not marble floors in office
   - Temporal consistency: Materials don't flicker frame-to-frame

3. **How much randomization before diminishing returns?**
   - 10 material variations: High benefit (covers common cases)
   - 100 material variations: Moderate benefit (covers edge cases)
   - 1000 material variations: Low benefit (data collection overhead exceeds generalization gain)
   - Guideline: 10-30 variations per attribute for most tasks

4. **What randomization frequency balances diversity vs temporal consistency?**
   - Static images: Randomize every frame (maximum diversity)
   - Video tracking: Randomize every 10-30 frames (temporal coherence)
   - Reinforcement learning episodes: Randomize once per episode (consistent environment)

5. **How do I validate randomization quality (not too uniform, not too extreme)?**
   - Qualitative: Manual review of 100 samples (plausibility check)
   - Quantitative: Measure attribute distribution (should be roughly uniform, not skewed 90% to one value)
   - Annotation accuracy: Spot-check bounding boxes on randomized scenes (&gt;95% accurate)
```

### Step 3: Articulate Principles

**Domain Randomization Principles**:
```markdown
**Principle 1: Systematic Variation** (Not Random Chaos)
Define variation axes (texture, lighting, pose), then sample within each systematically:
- Texture: Select environment type → Sample materials consistent with type
- Lighting: Sample base intensity → Vary individual lights by ±15%
- Pose: Define valid spawn region → Sample positions + collision check

**Principle 2: Plausibility Constraint** (Preserve Real-World Distributions)
All randomized scenes must be physically plausible:
- No floating objects (all objects on floor, gravity respected)
- No impossible material combinations (grass floors in office)
- No extreme lighting (not pitch black or overexposed)
- Test: "Could this scene exist in real world?" If no, reject sample.

**Principle 3: Task-Specific Focus** (Vary What Matters)
Randomize features relevant to task:
- Object detection: Background clutter, occlusion, lighting (model must see objects in varied contexts)
- Depth estimation: Surface textures, reflectivity (affects depth sensor accuracy)
- Navigation: Floor friction, obstacle density (affects path planning)
- Don't randomize irrelevant features (ceiling color for ground robot, floor texture for aerial drone)

**Principle 4: Balance Realism vs Diversity**
Prefer 1,000 diverse realistic images over 10,000 images with implausible scenes:
- Unrealistic data harms training (model learns artifacts)
- Quality > quantity for sim-to-real transfer
- Guideline: 100 hand-reviewed samples → If 5+ are implausible, tighten constraints

**Principle 5: Correlation Management**
Sample correlated variables together, independent variables separately:
- Correlated: Floor-wall materials (must match environment type)
- Correlated: Lighting intensity-floor reflectivity (dark floors need bright lights)
- Independent: Object pose-object material (position and color are uncorrelated)
```

## Creating the Skill File

**.claude/skills/isaac-sim-domain-randomization/SKILL.md**:
```markdown
---
name: isaac-sim-domain-randomization
description: Systematic domain randomization patterns for robust synthetic training data
version: 1.0.0
author: [Your name]
created: 2025-12-17
applies_to: ["isaac-sim", "synthetic-data", "sim-to-real", "computer-vision"]
---

# Isaac Sim Domain Randomization Skill

## Persona

[Paste persona from Step 1]

## Questions

[Paste 5 questions from Step 2]

## Principles

[Paste 5 principles from Step 3]

## Usage Examples

### Example 1: Warehouse Object Detection Randomization

**Context**: Training object detection model for warehouse robots. Objects: boxes, pallets, forklifts.

**Task**: Define randomization strategy.

**Apply skill**:

**Question 1**: What variations increase robustness?
- Texture: Box materials (cardboard brown, white, plastic blue/red)
- Lighting: Warehouse lights (fluorescent, varying intensity)
- Pose: Box positions, orientations, stacking
- Occlusion: Some boxes partially occluded by others

**Question 2**: What correlations preserve realism?
- Floor-wall correlation: Concrete/epoxy floors with industrial walls
- Lighting-time correlation: Bright overhead (day shift) vs dim (night shift)
- Object-environment correlation: Boxes/pallets belong in warehouse

**Principle 1**: Systematic variation
```python
environment = "warehouse"
floor = sample(["concrete_aged", "concrete_polished", "epoxy_gray"])
walls = sample(warehouse_walls[floor])  # Correlated sampling
lighting_base = sample(3500, 5000)  # Industrial range
object_poses = random_scatter_2d(collision_check=True)
```

**Principle 2**: Plausibility constraint
```python
# Reject samples with implausible combinations
if floor == "grass":  # Not plausible in warehouse
    resample()
if any(light.intensity < 1000):  # Too dim for warehouse
    resample()
```

**Outcome**: 10,000 images with:
- 3 floor types, 6 wall types (correlated)
- Lighting intensity: 3500-5000 (realistic warehouse range)
- 50-100 boxes per scene (typical warehouse density)
- Validation: 98% plausibility, 3% model improvement over uncorrelated randomization

### Example 2: Avoiding Over-Randomization

**Context**: Object detection model accuracy plateaus at 85% despite increasing dataset from 10K to 50K images.

**Problem**: Suspect diminishing returns from excessive randomization.

**Apply skill**:

**Question 3**: How much randomization before diminishing returns?
- Current: 100 material variations, 20 lighting conditions, 1000 pose variations
- Principle 4: Quality > quantity

**Analysis**:
- Manual review of 100 samples: 15 are implausible (grass floors, extreme lighting)
- Guideline: &gt;5% implausibility → Tighten constraints

**Action**:
1. Reduce material library to 30 high-quality realistic materials
2. Constrain lighting to 3000-5000 (remove extremes)
3. Add plausibility rejection sampling

**Result**:
- Dataset shrinks to 20K images (higher quality)
- Model accuracy improves to 89% (fewer artifacts learned)
- Training time halved (smaller dataset)

### Example 3: Video Tracking Randomization

**Context**: Training video object tracking model. Need temporal consistency.

**Task**: Determine randomization frequency.

**Apply skill**:

**Question 4**: What frequency balances diversity vs consistency?
- Option A: Every frame (maximum diversity, but materials flicker)
- Option B: Every 30 frames (1 second at 30 FPS)
- Option C: Once per video (consistent but less diverse)

**Principle 5**: Correlation management
- Randomize materials once per video (temporal consistency)
- Randomize object motion every frame (tracking target)

**Implementation**:
```python
def randomize_per_video():
    # Once per video clip (30 seconds)
    floor_material = sample_material()
    lighting_config = sample_lighting()

def randomize_per_frame():
    # Every frame (object motion only)
    object_velocity = sample_velocity()
    object_trajectory = update_trajectory()
```

**Outcome**: Model tracks objects across lighting changes (learned from video diversity) without being confused by flickering materials.

## Integration with Other Skills

- **isaac-sim-performance skill**: This skill focuses on data diversity, performance skill focuses on generation speed. Use both together for efficient high-quality dataset generation.

## Validation

**Skill is working when**:
- Randomized scenes are plausible (&gt;95% pass manual review)
- Attribute distributions are roughly uniform (not skewed 90% to one material)
- Model trained on randomized data generalizes to real-world (sim-to-real gap &lt;10%)

**Skill needs refinement if**:
- &gt;5% of scenes are implausible (tighten plausibility constraints)
- Model accuracy doesn't improve with more randomized data (check for artifacts)
- Real-world performance poor despite good synthetic validation (randomization doesn't cover real-world variation)
```

**Save as**: `.claude/skills/isaac-sim-domain-randomization/SKILL.md`

## Using the Skill (Validation)

**Test on novel scenario**: Hotel lobby robot (not warehouse).

**Apply skill**:
```
Question 1: What variations increase robustness for hotel lobby?
- Furniture styles (modern, traditional, minimalist)
- Floor materials (carpet, marble, tile)
- Lighting (chandelier, wall sconces, natural window light)
- Guest presence/absence (occlusion variation)

Question 2: What correlations preserve realism?
- Floor-furniture correlation: Marble floors → traditional/luxury furniture
- Lighting-material correlation: Reflective marble → lower light intensity needed

Principle 1: Systematic variation
- Define "hotel_lobby" environment type
- Sample floor → walls → furniture (correlated)
- Sample lighting based on floor reflectivity

Principle 2: Plausibility constraint
- Reject: Concrete floors in luxury hotel lobby
- Reject: Warehouse fluorescents with marble floors
```

**Validation**: Generated dataset has plausible hotel lobby scenes. Skill generalized beyond warehouse.

## Exercise: Extend Skill with Weather Randomization

**Task**: Add outdoor weather randomization to skill.

**New question**:
```markdown
6. **How do I randomize environmental conditions for outdoor scenarios?**
   - Weather: Clear, overcast, rain, fog (affects lighting and visibility)
   - Time of day: Morning, noon, evening, night (affects shadows)
   - Seasonal: Summer, winter (affects vegetation, ground cover)
```

**New principle**:
```markdown
**Principle 6: Environmental Condition Correlation**
Weather, time of day, and lighting are correlated:
- Rain → Overcast sky → Low light intensity → Wet ground reflectivity
- Clear day → Bright sun → Hard shadows → Dry ground
- Don't mix: Sunny sky with rain, or night with bright overhead sun
```

**Success criteria**:
- [ ] Added weather question and principle to SKILL.md
- [ ] Tested on outdoor navigation scenario
- [ ] Weather conditions correlate correctly (no sunny rain)

## Try With AI

**Setup**: Open ChatGPT or Claude.

**Part 1: Skill Design Review**
```
I'm creating a domain randomization skill for Isaac Sim. Review my questions and principles. What common pitfalls am I not addressing?

[Paste your draft skill]
```

**Part 2: Skill Application**
```
Apply my domain-randomization skill to this scenario:
- Task: Training depth estimation for outdoor drone
- Environment: Parks and forests
- Randomization needs: Trees, ground textures, lighting

Skill: [Paste Questions 1-5]

What variations matter? What correlations should I preserve?
```

**Part 3: Debugging Implausible Scenes**
```
I generated 10,000 images using my randomization skill. Manual review found 8% have implausible combinations. Examples:
- Snow on ground with green trees
- Nighttime with bright overhead sun
- Desert cacti in forest scene

Which principle from my skill addresses this? How do I tighten constraints?
```

**Part 4: Convergence Check**
After using skill on 3+ scenarios:
- What diversity patterns did skill capture?
- Where did skill need extension (new questions/principles)?
- How much does skill reduce randomization debugging time?
- Is skill reusable for different robotics domains (manipulation, navigation, perception)?

**Safety note**: Excessive randomization can create training data that models can't learn from (too noisy). If model accuracy doesn't improve after 10K images, investigate data quality before generating more.
