---
id: 0002
title: Animated Robot with Cursor Tracking Eyes
stage: green
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: homepage-vectra-branding
branch: main
user: abdullah2127x
command: direct-request
labels: ["robot", "animation", "cursor-tracking", "svg", "react-hooks"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - book-source/src/pages/index.tsx
  - book-source/src/pages/index.module.css
tests:
  - npm run build (verified successful)
---

## Prompt

regenerate the hero as there should be animated robot that make the hero looks modern the robot should animate continuesly and also this should also make specfic animation may be face or eye according to the move of the cursot

## Response snapshot

Created an interactive AnimatedRobot component with cursor-tracking eyes:

1. **Robot Design (SVG)**:
   - Full humanoid body with head, torso, arms, legs
   - Glowing energy core in chest
   - Antenna with pulsing light
   - Joint circles for mechanical look
   - Gradient fills (green to cyan theme)

2. **Cursor Tracking Implementation**:
   ```typescript
   const handleMouseMove = useCallback((e: MouseEvent) => {
     const robotElement = document.getElementById('robot-container');
     const rect = robotElement.getBoundingClientRect();
     const robotCenterX = rect.left + rect.width / 2;
     const robotCenterY = rect.top + rect.height / 3;
     const deltaX = e.clientX - robotCenterX;
     const deltaY = e.clientY - robotCenterY;
     const maxMove = 8;
     const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
     const normalizedDistance = Math.min(distance / 300, 1);
     setEyePosition({ x: moveX, y: moveY });
   }, []);
   ```

3. **Animations Added**:
   - robotFloat: Continuous up/down floating motion
   - coreGlow: Pulsing energy core
   - armSwing: Subtle arm movement
   - legShift: Subtle leg movement
   - antennaLight: Blinking antenna
   - particleFloat: Floating particles around robot
   - Blinking effect every 3-5 seconds

4. **React Hooks Used**:
   - useState for eye position and blink state
   - useEffect for mouse event listeners and blink interval
   - useCallback for optimized event handler

## Outcome

- ✅ Impact: Interactive robot that engages users with cursor-following eyes and continuous animations
- 🧪 Tests: Build successful, animations render correctly
- 📁 Files: 2 files modified (added AnimatedRobot component and robot CSS animations)
- 🔁 Next prompts: Repository rename, Vectra branding
- 🧠 Reflection: Cursor tracking adds interactivity that makes the page feel alive and modern

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): npm run build - PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add more personality to robot (expressions, gestures)
