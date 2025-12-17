# Directory Structure: Physical AI & Humanoid Robotics

**Version:** 1.0.0
**Last Updated:** 2025-12-16

This document defines the authoritative file organization for the book content.

---

## Book Content Structure

```
book-source/
├── docs/
│   ├── intro.md                           # Book introduction
│   │
│   ├── Part-1-ROS2-Foundation/            # Part 1: The Robotic Nervous System
│   │   ├── _category_.json                # Part metadata for Docusaurus
│   │   ├── index.md                       # Part introduction
│   │   │
│   │   ├── 01-ros2-nodes-topics-services/ # Chapter 01
│   │   │   ├── _category_.json
│   │   │   ├── index.md                   # Chapter overview
│   │   │   ├── 01-introduction.md         # Lesson 1
│   │   │   ├── 02-nodes.md                # Lesson 2
│   │   │   ├── 03-topics.md               # Lesson 3
│   │   │   ├── ...                        # Additional lessons
│   │   │   └── exercises.md               # Chapter exercises
│   │   │
│   │   ├── 02-rclpy-python-agents/        # Chapter 02
│   │   │   ├── _category_.json
│   │   │   ├── index.md
│   │   │   └── ...
│   │   │
│   │   └── 03-urdf-humanoids/             # Chapter 03
│   │       ├── _category_.json
│   │       ├── index.md
│   │       └── ...
│   │
│   ├── Part-2-Digital-Twin/               # Part 2: The Digital Twin
│   │   ├── _category_.json
│   │   ├── index.md
│   │   │
│   │   ├── 04-gazebo-physics/             # Chapter 04
│   │   │   └── ...
│   │   │
│   │   ├── 05-unity-hri/                  # Chapter 05
│   │   │   └── ...
│   │   │
│   │   └── 06-sensors-simulation/         # Chapter 06
│   │       └── ...
│   │
│   ├── Part-3-Isaac-Brain/                # Part 3: The AI-Robot Brain
│   │   ├── _category_.json
│   │   ├── index.md
│   │   │
│   │   ├── 07-isaac-sim-synthetic-data/   # Chapter 07
│   │   │   └── ...
│   │   │
│   │   ├── 08-isaac-ros-vslam/            # Chapter 08
│   │   │   └── ...
│   │   │
│   │   └── 09-nav2-bipedal/               # Chapter 09
│   │       └── ...
│   │
│   └── Part-4-VLA-Integration/            # Part 4: Vision-Language-Action
│       ├── _category_.json
│       ├── index.md
│       │
│       ├── 10-voice-whisper/              # Chapter 10
│       │   └── ...
│       │
│       ├── 11-llm-cognitive-planning/     # Chapter 11
│       │   └── ...
│       │
│       └── 12-capstone-autonomous/        # Chapter 12
│           └── ...
│
├── static/
│   ├── img/
│   │   ├── parts/                         # Part-level images
│   │   └── chapters/                      # Chapter-level images
│   └── code/
│       └── examples/                      # Downloadable code examples
│
└── src/
    └── css/
        └── custom.css                     # Custom styling
```

---

## Naming Conventions

### Parts
- **Directory:** `Part-X-Short-Name/` (capitalized, hyphenated)
- **Example:** `Part-1-ROS2-Foundation/`

### Chapters
- **Directory:** `NN-slug-name/` (numbered, lowercase, hyphenated)
- **Example:** `01-ros2-nodes-topics-services/`

### Lessons
- **File:** `NN-slug-name.md` (numbered, lowercase, hyphenated)
- **Example:** `01-introduction.md`, `02-nodes.md`

### Images
- **Path:** `static/img/chapters/NN/image-name.png`
- **Example:** `static/img/chapters/01/ros2-node-diagram.png`

---

## Required Files

### Part Level
- `_category_.json` - Docusaurus sidebar configuration
- `index.md` - Part introduction with learning outcomes

### Chapter Level
- `_category_.json` - Docusaurus sidebar configuration
- `index.md` - Chapter overview with objectives
- Lesson files (numbered sequentially)
- `exercises.md` - Hands-on exercises (optional, can be inline)

---

## Category JSON Templates

### Part Category (`Part-X-.../_category_.json`)
```json
{
  "label": "Part 1: The Robotic Nervous System",
  "position": 2,
  "link": {
    "type": "doc",
    "id": "Part-1-ROS2-Foundation/index"
  }
}
```

### Chapter Category (`NN-chapter/.../_category_.json`)
```json
{
  "label": "Chapter 1: ROS 2 Nodes, Topics, and Services",
  "position": 1,
  "link": {
    "type": "doc",
    "id": "Part-1-ROS2-Foundation/01-ros2-nodes-topics-services/index"
  }
}
```

---

## Validation Rules

1. **Numbered Sequencing:** All chapters and lessons must be numbered
2. **Index Files:** Every directory must have an `index.md`
3. **Category Files:** Every directory must have `_category_.json`
4. **No Orphan Files:** All content files must be in proper directory hierarchy
5. **Frontmatter Required:** All `.md` files must have YAML frontmatter

---

## Frontmatter Template

```yaml
---
sidebar_position: 1
title: "Lesson Title"
description: "Brief description for SEO and navigation"
---
```
