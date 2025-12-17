---
title: "Lesson 3: Environment Design and Lighting"
chapter: 5
lesson: 3
proficiency_level: B2
learning_objectives:
  - "Design photorealistic indoor environments"
  - "Implement three-point lighting for professional appearance"
  - "Apply materials and textures effectively"
  - "Optimize performance for real-time rendering"
  - "Collaborate with AI on design decisions"
estimated_time: "120 minutes"
generated_by: content-implementer v1.0.0
created: 2025-12-16
version: 1.0.0
---

# Lesson 3: Environment Design and Lighting

## Introduction

The humanoid model is visible but isolated in empty space. Now create the world around it: a photorealistic indoor environment where human-robot interaction happens.

This lesson demonstrates how AI helps optimize design decisions. You'll propose an environment, AI suggests lighting and material approaches, you provide constraints (performance targets), and together you arrive at a professional-looking scene that renders smoothly.

Think of this as collaborative design—neither you nor AI has the complete solution alone, but through iteration, a polished environment emerges.

**Estimated time**: 120 minutes
**Concept density**: 6 new concepts (within B2 limit)

---

## The Design Challenge

Create a scene where humans and robots interact naturally. You must balance:
- **Realism** — Photorealistic for credible research
- **Performance** — 60+ FPS for interactive simulation
- **Clarity** — Humanoid visible and interaction spaces obvious

This requires design decisions you haven't made before. This is where AI collaboration becomes valuable.

---

## Concept 1: Environment Scenarios

Choose your environment scenario. Each affects what you build:

**Option A: Living Room** (residential setting)
- Furniture: Sofa, coffee table, shelves, TV
- Lighting: Warm (evening lamp + window light)
- Flooring: Hardwood or carpet
- Size: ~4m x 6m
- Interaction: Human sits, robot brings object

**Option B: Office** (workplace setting)
- Furniture: Desk, chairs, whiteboard, plants
- Lighting: Overhead fluorescent + window light
- Flooring: Tile or carpet
- Size: ~3m x 4m
- Interaction: Human at desk, robot assists with tasks

**Option C: Warehouse** (industrial setting)
- Furniture: Shelves, boxes, pallets
- Lighting: Industrial fluorescents + skylights
- Flooring: Concrete
- Size: ~8m x 10m
- Interaction: Human and robot navigate cluttered space

Pick one. For this lesson, we'll design **Office** (good balance of complexity and clarity).

---

## Concept 2: Three-Point Lighting

Professional lighting uses three light sources:

### Light 1: Key Light (Main)
- **Purpose**: Primary illumination
- **Characteristics**: Bright, warm (usually), creates main shadows
- **Position**: 45° to side, slightly above eye level
- **Intensity**: Strongest light source

### Light 2: Fill Light (Secondary)
- **Purpose**: Fills shadows, shows detail
- **Characteristics**: Softer, secondary color (often cooler)
- **Position**: Opposite key light, slightly lower
- **Intensity**: 50-75% of key light intensity

### Light 3: Back Light (Rim)
- **Purpose**: Separates subject from background
- **Characteristics**: Often cooler color, creates rim highlight
- **Position**: Behind subject, pointed at camera
- **Intensity**: 25-50% of key light

**Implementation in Unity**:

Create three Directional Lights in your scene:

```csharp
// Pseudo-code structure for three-point lighting
KeyLight = new DirectionalLight()
{
    Intensity = 1.0f,
    Color = Color.yellow,  // warm
    Rotation = Euler(45, 45, 0)
};

FillLight = new DirectionalLight()
{
    Intensity = 0.6f,
    Color = Color.cyan,  // cool
    Rotation = Euler(45, -135, 0)
};

BackLight = new DirectionalLight()
{
    Intensity = 0.3f,
    Color = Color.white,
    Rotation = Euler(135, 0, 0)
};
```

---

## Concept 3: Scene Composition

Organize your scene hierarchy clearly:

```
Office Scene
├── Lighting
│   ├── Key Light (Directional)
│   ├── Fill Light (Directional)
│   ├── Back Light (Directional)
│   └── Window Light (optional Point light)
├── Environment
│   ├── Walls (simple cubes or imported models)
│   ├── Floor (plane with carpet texture)
│   ├── Ceiling (simple plane)
│   ├── Furniture
│   │   ├── Desk (imported model or modeled)
│   │   ├── Chair (imported)
│   │   └── Shelves (modeled)
│   └── Decorations
│       ├── Plants
│       └── Wall art
├── Actors
│   ├── Humanoid (from Lesson 2)
│   └── Human Avatar (coming Lesson 4)
└── Interaction Volume (invisible trigger)
```

**Best Practice**: Group objects by function. Lighting separate from environment. Actors separate from scene.

---

## Concept 4: Materials and Textures

Materials define how surfaces look:

**Material Components**:
- **Albedo** — Base color (what color is it?)
- **Metallic** — Metal (0=matte, 1=shiny)
- **Smoothness** — Surface finish (0=rough, 1=glossy)
- **Normal Map** — Surface detail (bumps, scratches)
- **Emission** — Self-glowing color

**Material Recipes for Office**:

```
Walls:
  Albedo: Light gray (200, 200, 200)
  Metallic: 0
  Smoothness: 0.2 (slight texture)

Floor (Carpet):
  Albedo: Medium gray (150, 150, 150)
  Metallic: 0
  Smoothness: 0.1 (rough fabric)

Desk (Wood):
  Albedo: Brown (140, 100, 60)
  Metallic: 0
  Smoothness: 0.4 (polished wood)

Metal Shelves:
  Albedo: Dark gray (80, 80, 80)
  Metallic: 0.8
  Smoothness: 0.6 (anodized aluminum)
```

---

## Concept 5: Performance Optimization

Real-time rendering at 60+ FPS requires optimization:

### Draw Calls
Each visible mesh = one draw call. Too many = performance drop.

**Optimization**: Static Batching
- Mark static objects: Inspector → Static Batching ✅
- Unity combines multiple meshes into single draw call
- Only works for non-moving objects

### Level of Detail (LOD)
Distant objects use simpler meshes.

**Optimization**: LOD Groups
- Select mesh → Add Component → LOD Group
- Define levels: LOD0 (high detail), LOD1 (medium), LOD2 (low)
- Unity automatically switches based on distance

### Culling
Don't render objects outside camera view.

**Optimization**: Occlusion Culling
- Window → Rendering → Occlusion Culling
- Bake scene → Unity skips rendering hidden objects

---

## Designing for Performance and Quality

**Initial approach:**
Creating a realistic office with high-detail textures, imported furniture, and professional lighting.

**Design considerations:**
Three-point lighting creates professional appearance. Material variation (matte walls, glossy desk, textured carpet) adds realism. But high detail everywhere creates performance cost.

For 60+ FPS with humanoid animation, strategic detail placement works better:
- High detail near camera (where human/robot interact)
- Low detail in background
- Baked lighting instead of real-time for static objects

**Refinement based on constraints:**
"I need 60+ FPS for smooth interaction. Should I avoid imported furniture models entirely?"

Not necessarily. Use imported models for key furniture (desk, chair—focal points users see up close). Simplify background objects:
- Desk: High-poly imported model
- Shelves: Simple modeled boxes (background, less focus)
- Boxes on shelves: Procedurally generated

**Finding the balance through iteration:**

**Iteration 1**: High-detail everything
- Result: 25 FPS (too slow for interaction)

**Iteration 2**: Reduce detail uniformly
- Result: 55 FPS (better, but environment looks cheap)

**Iteration 3**: Strategic detail + baked lighting
- Desk high-detail (focal point), background low-detail
- Bake static lighting instead of real-time
- Result: 60+ FPS AND professional appearance

This approach emerged through testing different configurations and measuring their impact.

---

## Hands-On: Build Your Office Environment

### Step 1: Scene Setup (15 minutes)

Create new scene: **File → New Scene → 3D (Standard)**

Save as `Scenes/OfficeHRI.unity`

Add ground plane:
1. **Hierarchy → Create Empty → Rename to "Environment"**
2. **Create → Plane → Rename to "Floor"**
3. Scale: (4, 1, 6) — office-sized
4. Position: (0, 0, 0)

### Step 2: Three-Point Lighting (15 minutes)

1. **Hierarchy → Light → Directional Light → Rename "KeyLight"**
   - Position: (3, 3, -2)
   - Intensity: 1.2
   - Color: (255, 240, 200) — warm

2. **Create another Directional Light → Rename "FillLight"**
   - Position: (-3, 2, 2)
   - Intensity: 0.6
   - Color: (100, 150, 200) — cool blue

3. **Create another Directional Light → Rename "BackLight"**
   - Position: (0, 2, 3)
   - Intensity: 0.3
   - Color: (255, 255, 255) — white

Press Play. Humanoid should be well-lit from multiple angles.

### Step 3: Furniture (20 minutes)

Create simple furniture using basic shapes:

**Desk** (using cubes):
```
Create → Cube → Rename "Desk"
Position: (1, 0.5, -1)
Scale: (2, 1, 1)  // long table
Material: Wood color
```

**Chair** (using capsule):
```
Create → Capsule → Rename "Chair"
Position: (1, 0.5, 0.5)
Scale: (0.5, 1, 0.5)
Material: Fabric (medium gray)
```

**Shelves** (using cubes):
```
Create → Cube → Rename "Shelf1"
Position: (-2, 1.5, 0)
Scale: (0.2, 1, 1)
Material: Metal (dark gray, high shine)
```

**Walls** (using planes):
```
Create → Plane → Rename "WallLeft"
Position: (-2, 1, 0)
Rotation: (90, 0, 0)
Scale: (1, 3, 1)
Material: Wall (light gray, matte)
```

### Step 4: Apply Materials (15 minutes)

Create materials for each type:
- Right-click Assets → Create → Material
- Name: `mat_wood`, `mat_metal`, `mat_wall`, `mat_carpet`
- Set colors and smoothness
- Drag onto corresponding GameObjects

### Step 5: Performance Check (10 minutes)

1. **Window → Analysis → Profiler**
2. Press Play
3. Watch CPU/GPU performance
4. Note frame rate (target 60+ FPS)

If below 60 FPS:
- Reduce detail (fewer objects, simpler meshes)
- Enable Static Batching (select furniture → Inspector → Static ✅)
- Use LOD Groups on distant objects

### Step 6: Position Humanoid (5 minutes)

1. Drag Humanoid prefab into scene
2. Position: (0, 0, 0.5) — center of office space
3. Verify it's well-lit and visible

### Checkpoint: Professional-Looking Office

Verify:
- ✅ Scene has organized hierarchy
- ✅ Three-point lighting visible (bright key, softer fill, rim separation)
- ✅ Furniture positioned logically
- ✅ Materials applied with color variation
- ✅ Humanoid visible and well-lit
- ✅ Performance at 60+ FPS

---

## Try With AI

**Exploration 1: Lighting Recipes**

Ask AI: "Generate 3 different three-point lighting recipes for different moods: professional office, warm residential, industrial warehouse. Show the color and intensity for each."

Implement each lighting setup:
- Save your current office scene
- Create new scenes for residential and warehouse
- Apply AI-suggested lighting
- Compare visual appearance and performance

**Exploration 2: Environmental Design for User Studies**

Ask AI: "If you were designing an environment for a human-robot interaction study, what environmental factors would influence how humans perceive robot behavior? How would you control those factors?"

Discuss:
- Lighting affects perception of robot threat/comfort
- Furniture arrangement affects personal space
- Color psychology influences emotional response

Try redesigning your office:
- Test warm lighting vs cool lighting on how humanoid appears
- Test cluttered vs minimal arrangement on interaction comfort

---

[Next: Lesson 4 - Human Avatar Animation](04-human-avatars.md)
