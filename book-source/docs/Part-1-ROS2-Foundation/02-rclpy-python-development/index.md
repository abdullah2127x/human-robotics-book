---
sidebar_position: 2
title: "Chapter 2: Bridging Python Agents to ROS Controllers"
description: "Master rclpy for building Python-based robot controllers with async patterns, actions, and custom messages"
---

# Chapter 2: Bridging Python Agents to ROS Controllers (rclpy)

In Chapter 1, you learned to create ROS 2 nodes that communicate through topics and services. Now you'll build sophisticated Python agents that can control robots autonomously using the full power of rclpy.

## What You'll Learn

This chapter covers the Python client library (rclpy) in depth:

- **Package Structure** — Organize ROS 2 Python code professionally
- **Async Callbacks** — Write non-blocking code for responsive robots
- **Actions** — Handle long-running tasks with progress feedback
- **Custom Messages** — Define structured data for robot communication
- **Executors** — Control concurrent callback execution
- **Reusable Patterns** — Extract templates for faster development

## Chapter Structure

| Lesson | Topic | Focus |
|--------|-------|-------|
| 1 | Package Structure | ROS 2 Python package organization |
| 2 | Async/Await | Non-blocking callback patterns |
| 3 | Action Clients | Long-running tasks with feedback |
| 4 | Custom Messages | Structured robot communication |
| 5 | Executors | Single vs multi-threaded execution |
| 6 | Reusable Patterns | Template extraction and code reuse |
| 7 | Capstone | Python agent controlling robot |

## Prerequisites

- Chapter 1 complete (nodes, topics, services)
- Python 3.10+ familiarity
- Basic async/await understanding helpful

## By the End

You'll build an autonomous Python agent that:
- Subscribes to sensor data
- Makes decisions based on conditions
- Commands robot movement via actions
- Reports status through custom messages
- Runs responsively with async callbacks

Let's start with how ROS 2 organizes Python code into packages.
