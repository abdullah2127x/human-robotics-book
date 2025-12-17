# Lesson 8: Creating Performance Optimization Skill

You've built complete Isaac Sim pipelines (rendering, randomization, Replicator, ROS 2 bridge). But each project required performance tuning—adjusting render settings, batch sizes, and memory management to hit target FPS.

Now you'll extract those optimization patterns into a **reusable skill**: the isaac-sim-performance skill. Future projects can invoke this skill instead of rediscovering optimizations from scratch.

## What is a Skill?

**Skill** (from AI-native development paradigm): A structured intelligence artifact encoding expertise as **Persona + Questions + Principles**.

**Structure**:
```markdown
**Persona**: Cognitive stance that activates the right thinking
**Questions**: Analysis questions forcing context-specific reasoning
**Principles**: Decision frameworks guiding application
```

**Why skills?** They're more powerful than code libraries—they encode reasoning patterns, not just implementations.

## Reviewing Performance Patterns from Lessons 4-7

**Lesson 4** (Rendering): RTX samples/bounces tradeoff → Found 64 samples, 3 bounces optimal for 30 FPS real-time generation

**Lesson 6** (Replicator): Batch rendering → Discovered 4 cameras in parallel = 3x speedup

**Lesson 7** (ROS 2 Bridge): Publication rate limiting → Learned throttling to 30 Hz prevents subscriber lag

**Common pattern**: Profile bottleneck → Identify tunable parameters → Test configurations → Converge on optimal settings for hardware constraints.

## Designing the Performance Skill

### Step 1: Define Persona

**Think like real-time rendering engineer** who optimizes GPU utilization.

**Persona statement**:
```markdown
Think like a real-time rendering engineer optimizing GPU-accelerated simulation for maximum throughput while staying within hardware constraints (VRAM, GPU compute, memory bandwidth). Your goal is to maximize simulation speed and rendering quality without GPU out-of-memory (OOM) errors or unacceptable visual degradation.
```

**Why this persona?** It establishes the optimization objective (speed + quality balance) and constraints (hardware limits). This prevents over-optimization (sacrificing all quality) or under-optimization (ignoring performance).

### Step 2: Formulate Analysis Questions

**Questions force systematic reasoning** instead of random parameter tweaking.

**Performance Skill Questions**:
```markdown
1. **What's the performance bottleneck?** (Profile first, don't guess)
   - GPU compute-bound? (utilization near 100%)
   - GPU memory-bound? (VRAM usage near capacity)
   - CPU-bound? (GPU waiting for CPU to prepare data)
   - I/O-bound? (loading textures from disk)

2. **Which render settings impact quality vs speed?**
   - Samples per pixel: Linear speedup, affects noise level
   - Max bounces: Linear speedup, affects indirect illumination
   - Resolution: Quadratic speedup (halving resolution = 4x faster)
   - Denoising: Minimal cost, compensates for lower samples

3. **What LOD (Level of Detail) strategies apply?**
   - Distant objects: Use low-poly meshes (far from camera)
   - Off-screen culling: Don't render what camera can't see
   - Instancing: Reuse geometry for repeated objects (boxes, trees)

4. **How do I balance batch size for GPU memory?**
   - Larger batches → Faster (parallel processing)
   - Risk: OOM if batch exceeds VRAM capacity
   - Formula: batch_size = f(VRAM_free, texture_size, num_cameras)

5. **When should I use RTX vs rasterization?**
   - RTX (ray tracing): Photorealistic, 2-3x slower, essential for sim-to-real
   - Rasterization: Fast, less realistic, good for debugging/testing
   - Decision: Training data → RTX required. Functional testing → Rasterization sufficient.
```

**Why these questions?** They cover the full optimization space encountered in Lessons 4-7: render quality, memory management, rendering mode selection.

### Step 3: Articulate Principles

**Principles are decision frameworks**, not rules.

**Performance Optimization Principles**:
```markdown
**Principle 1: Profile First** (Don't Guess Bottlenecks)
Use nvidia-smi, Isaac Sim profiler, or nsys to identify bottleneck before optimizing.
- If GPU-Util < 80%: Bottleneck is elsewhere (CPU, I/O), don't tune render settings
- If GPU memory near 100%: Reduce batch size or texture resolution
- If GPU-Util ~100%: Reduce render quality (samples, bounces, resolution)

**Principle 2: LOD for Distant Objects**
High-poly meshes for objects within 5m of camera, low-poly beyond 10m.
- Performance gain: 30-50% in large scenes
- Quality impact: Minimal (distant objects occupy few pixels)

**Principle 3: Culling Off-Screen Geometry**
Enable view frustum culling (don't render what camera can't see).
- Isaac Sim default: Enabled
- Verify: Check render stats for culled prim count

**Principle 4: Batch Size = f(VRAM)**
Empirically determine max batch size for your GPU:
- 8GB VRAM: Batch size 4-8 cameras
- 12GB VRAM: Batch size 8-16 cameras
- 24GB VRAM: Batch size 16-32 cameras
- If OOM: Reduce by 50%, test again

**Principle 5: Graceful Degradation**
If GPU overloads mid-generation, automatically reduce quality before crashing:
- Drop samples: 64 → 32
- Drop resolution: 1280x720 → 640x480
- Continue generation at lower quality (better than crash + restart)
```

**Why principles, not rules?** Context matters. Principle 4 gives rough guidelines but expects you to measure your specific workload.

## Creating the Skill File

**Skill format** (canonical from book patterns):

**Directory structure**:
```
.claude/skills/isaac-sim-performance/
├── SKILL.md (main skill document)
└── examples/ (optional usage examples)
```

**SKILL.md template**:
```markdown
---
name: isaac-sim-performance
description: Performance optimization patterns for Isaac Sim real-time rendering and synthetic data generation
version: 1.0.0
author: [Your name]
created: 2025-12-17
applies_to: ["isaac-sim", "omniverse", "gpu-rendering", "synthetic-data"]
---

# Isaac Sim Performance Optimization Skill

## Persona

Think like a real-time rendering engineer optimizing GPU-accelerated simulation for maximum throughput while staying within hardware constraints (VRAM, GPU compute, memory bandwidth). Your goal is to maximize simulation speed and rendering quality without GPU out-of-memory errors or unacceptable visual degradation.

## Questions

[Paste 5 questions from Step 2]

## Principles

[Paste 5 principles from Step 3]

## Usage Examples

### Example 1: Optimizing Synthetic Data Generation

**Context**: Generating 10,000 images at 1280x720 with RTX rendering on RTX 3070 (8GB VRAM).

**Problem**: Generation takes 10 hours (needs &lt;4 hours for overnight run).

**Apply skill**:
1. **Profile**: nvidia-smi shows GPU-Util ~60%, Memory-Usage 7GB/8GB → Memory-bound
2. **Question 4**: Batch size limited by VRAM
3. **Principle 4**: RTX 3070 (8GB) → Max batch 4-6 cameras
4. **Action**: Currently using 8 cameras (exceeds guideline), reduce to 4
5. **Result**: Generation time drops to 6 hours (faster due to fewer OOM retries)

**Further optimization**:
6. **Question 2**: Reduce resolution? 1280x720 → 640x480 = 4x speedup
7. **Constraint check**: Model trains on 512x512 anyway (downsamples from 1280x720)
8. **Action**: Generate at 640x480 directly (closer to training resolution)
9. **Result**: Generation time drops to 3.5 hours (meets target)

### Example 2: Debugging Low FPS in ROS 2 Bridge

**Context**: Camera publishes at 15 FPS (target: 30 FPS) when ROS 2 bridge active.

**Problem**: Need to identify bottleneck (rendering vs bridge vs network).

**Apply skill**:
1. **Principle 1**: Profile first
2. **Test isolation**: Disable ROS 2 bridge → FPS jumps to 45
3. **Conclusion**: Bridge is bottleneck (not rendering)
4. **Question 1**: What's bottleneck? (Answer: ROS 2 message serialization)
5. **Principle 5**: Graceful degradation → Reduce publication rate to match processing speed
6. **Action**: Throttle camera to 30 Hz (ROS 2 can handle this)
7. **Result**: Stable 30 FPS with bridge active

## Integration with Other Skills

- **domain-randomization skill**: Both skills tune performance, but domain-randomization focuses on data diversity, this skill focuses on rendering speed
- **ros2-bridge skill** (future): This skill identifies when bridge is bottleneck, ros2-bridge skill optimizes bridge configuration

## Validation

**Skill is working when**:
- You systematically profile before optimizing (not guessing)
- You can articulate which principle applies to your bottleneck
- You achieve target FPS without quality degradation below acceptable threshold

**Skill needs refinement if**:
- You're still randomly tweaking parameters hoping for improvement
- Optimizations work on one GPU but fail on another (need more adaptive principles)
```

**Save as**: `.claude/skills/isaac-sim-performance/SKILL.md`

## Using the Skill (Validation)

**Test skill on novel scenario** (not from Lessons 4-7):

**New scenario**: Generating depth maps (not RGB) at 60 FPS for real-time SLAM.

**Apply skill**:
```
You (reading skill): "What's the performance bottleneck?"
[Run profiler]
You: "GPU-Util = 95%, VRAM = 4GB/8GB → Compute-bound"

You: "Which render settings impact quality vs speed for depth?"
[Consult Principle 2]
Skill: "Depth doesn't need ray tracing (only distance). Use rasterization mode."

You: "When should I use RTX vs rasterization?"
[Consult Question 5]
Skill: "Rasterization sufficient for depth (geometric information, not photorealism)."

Action: Switch to rasterization mode
Result: FPS jumps from 28 to 90 (3x speedup, depth accuracy unchanged)
```

**Validation**: Skill helped you discover rasterization mode works for depth (you hadn't considered this). Skill generalized beyond RGB lessons.

## Exercise: Extend the Skill

**Task**: Add texture streaming optimization to the skill.

**Problem**: Loading 20 high-res textures (2GB each) causes VRAM OOM. But most textures aren't visible simultaneously (camera sees 5-7 materials per frame).

**Your addition**:

**New Question**:
```markdown
6. **How do I manage texture memory for large material libraries?**
   - Load all textures upfront: Simple but risks OOM
   - Stream textures on-demand: Load only visible materials
   - Texture compression: Reduce VRAM footprint (minor quality loss)
```

**New Principle**:
```markdown
**Principle 6: Texture Streaming for Large Libraries**
If material library &gt;10GB, enable texture streaming:
- Isaac Sim loads visible textures only (unloads off-screen)
- Trade-off: Initial load latency (100-200ms when material first appears)
- Acceptable for static cameras, problematic for fast camera motion
```

**Success criteria**:
- [ ] Added new question and principle to SKILL.md
- [ ] Tested on scenario with 50+ textures
- [ ] Validated texture streaming reduces VRAM usage by 60%+

## Try With AI

**Setup**: Open ChatGPT or Claude.

**Part 1: Skill Design Review**
Ask AI:
```
I'm creating a performance optimization skill for Isaac Sim. Review my persona, questions, and principles. What's missing? What optimization patterns did I not cover?

[Paste your draft skill]
```

Incorporate AI's suggestions.

**Part 2: Skill Application**
Tell AI a new scenario:
```
I'm generating 100,000 images (training dataset) on an RTX 4090 (24GB VRAM). Currently takes 18 hours. I need &lt;12 hours for daily runs. Apply the isaac-sim-performance skill to optimize.

Skill: [Paste your skill's Questions and Principles]
My profile data: GPU-Util 85%, VRAM 20GB/24GB, CPU 40%, Disk I/O 60%
```

Does AI correctly identify bottleneck using your skill?

**Part 3: Skill Validation**
After using skill on 3+ scenarios:
- What patterns did skill successfully capture?
- Where did skill need refinement (missing principles)?
- How much faster is optimization with skill vs without?
- Is this skill reusable for teammates or future projects?

**Safety note**: GPU overclocking or extreme optimization can cause hardware instability. Stay within manufacturer specifications. If GPU crashes during generation, reduce settings—don't push harder.
