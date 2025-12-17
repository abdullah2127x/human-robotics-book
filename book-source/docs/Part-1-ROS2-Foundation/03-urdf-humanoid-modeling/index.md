---
sidebar_position: 3
title: "Chapter 3: Understanding URDF for Humanoids"
description: "Model humanoid robots in URDF with links, joints, and inertial properties for physics simulation"
---

# Chapter 3: Understanding URDF for Humanoids

Chapters 1-2 taught you to create ROS 2 nodes and Python agents. Now you'll learn to describe the physical structure of robots using URDF (Unified Robot Description Format)—the foundation for simulation and control.

## What You'll Learn

This chapter covers robot modeling with URDF:

- **Links** — Define rigid body segments with geometry
- **Joints** — Connect links with movement constraints
- **Inertial Properties** — Calculate mass and inertia for physics
- **Transform Frames** — Understand coordinate relationships
- **Xacro Macros** — Create reusable model templates

## Chapter Structure

| Lesson | Topic | Focus |
|--------|-------|-------|
| 1 | URDF Fundamentals | XML structure and single links |
| 2 | Collision & Inertial | Physics-ready link properties |
| 3 | Joints | Connecting links with constraints |
| 4 | Inertia Calculations | Formulas for common shapes |
| 5 | Transform Frames | Coordinate frame hierarchy |
| 6 | Articulated Arm | Multi-joint robot arm |
| 7 | Xacro Patterns | Reusable macros and templates |
| 8 | Capstone | Complete humanoid upper body |

## Prerequisites

- Chapter 1-2 complete (ROS 2 basics, Python agents)
- Basic geometry understanding
- Familiarity with XML helpful

## By the End

You'll build a complete humanoid upper body with:
- Torso and head with proper geometry
- Two articulated arms (shoulder, elbow, wrist)
- Calculated inertial properties for physics
- Reusable xacro macros for efficiency
- Validated model ready for simulation

This model will be used in Part 2 for physics simulation in Gazebo.

Let's start with the fundamentals of URDF structure.
