---
sidebar_position: 1
title: "Chapter 1: ROS 2 Nodes, Topics, and Services"
description: "Introduction to ROS 2 architecture - the nervous system that connects all robotic components"
---

# Chapter 1: ROS 2 Nodes, Topics, and Services

**Part 1: The Robotic Nervous System** | **Cognitive Load: Light** | **Tier: B1**

---

## Chapter Overview

Welcome to the foundation of robotic systems. In this chapter, you'll learn how ROS 2 orchestrates communication between all components of a robot - from sensors to actuators, from perception to planning.

Think of ROS 2 as the nervous system of a robot. Just as neurons communicate through electrical signals, ROS 2 nodes communicate through messages. Master this, and you understand how robots "think."

---

## What You'll Learn

By the end of this chapter, you will be able to:

- [ ] **LO-1.1:** Explain the ROS 2 node lifecycle and architecture
- [ ] **LO-1.2:** Create and run ROS 2 nodes using Python (rclpy)
- [ ] **LO-1.3:** Implement publishers and subscribers for topic-based communication
- [ ] **LO-1.4:** Create and call ROS 2 services for request-response patterns
- [ ] **LO-1.5:** Configure Quality of Service (QoS) policies
- [ ] **LO-1.6:** Debug ROS 2 systems using command-line tools

---

## Key Concepts

| Concept | Description |
|---------|-------------|
| **Node** | An independent process that performs computation |
| **Topic** | A named bus for asynchronous, many-to-many messaging |
| **Publisher** | A node that sends messages to a topic |
| **Subscriber** | A node that receives messages from a topic |
| **Service** | Synchronous request-response communication |
| **QoS** | Quality of Service policies for reliable communication |

---

## Hands-On Exercises

Throughout this chapter, you will:

1. **Create your first ROS 2 node** - A simple "Hello Robot" publisher
2. **Build a pub/sub system** - Temperature sensor simulation
3. **Implement a service** - Robot status query system
4. **Debug with CLI tools** - `ros2 topic`, `ros2 node`, `ros2 service`

---

## Prerequisites

Before starting this chapter:

- [ ] Ubuntu 22.04 LTS installed (or WSL2)
- [ ] ROS 2 Humble installed
- [ ] Python 3.10+ configured
- [ ] Basic Python knowledge

---

## Chapter Outline

All lessons are ready! Work through them in order:

1. **[Lesson 1: Introduction to ROS 2 Nodes](01-introduction.md)** - Create your first node
2. **[Lesson 2: Topics and Pub/Sub Communication](02-understanding-nodes.md)** - Connect nodes together
3. **[Lesson 3: Debugging with ROS 2 CLI Tools](03-topics-and-messages.md)** - Inspect running systems
4. **[Lesson 4: Services and Synchronous Requests](04-publishers-and-subscribers.md)** - Request-response patterns
5. **[Lesson 5: Quality of Service (QoS)](05-services-and-clients.md)** - Reliability and performance tradeoffs
6. **[Lesson 6: Reusable Node Patterns](06-quality-of-service.md)** - Design for code reuse
7. **[Lesson 7: Capstone Project - Multi-Node Robot System](07-capstone-integration.md)** - Build a complete system

---

## Connection to Next Chapters

This chapter prepares you for:

- **Chapter 2:** Use these concepts to build Python agents with rclpy
- **Chapter 3:** URDF models are loaded and visualized through ROS 2 topics
- **Part 2+:** All simulation and control builds on ROS 2 communication

---

## Ready to Begin?

Start with Lesson 1: Introduction to ROS 2 Architecture.
