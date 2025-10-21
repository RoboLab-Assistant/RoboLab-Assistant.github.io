<p align="center">
  <img src="docs/logos/RoboLab Assistant - Square (Dark).svg" alt="RoboLab Assistant logo" width="140">
</p>

<p align="center">
  <!-- Theme-aware wordmark on GitHub -->
  <img src="docs/logos/wordmark-dark.svg#gh-dark-mode-only" width="520" alt="RoboLab Assistant">
  <img src="docs/logos/wordmark-light.svg#gh-light-mode-only" width="520" alt="RoboLab Assistant">
</p>

<p align="center">
  A vision and motion planning framework for robotic arms that moves test tubes (cuvettes) safely for chemical testing.
</p>

---

## What is it?

RoboLab Assistant is a student-built system that helps automate a common lab task: placing a cuvette into an analyser, closing the lid, and starting a run. It focuses on reliability, clear safety behaviour, and a workflow that is easy to understand and demo.

## Highlights

- 👀 **Knows where things are** using a small wrist camera and simple markers
- 🤖 **Handles gently** with a UR3e arm and a tuned gripper
- 🧭 **Chooses safe moves** on a crowded bench, then executes step by step
- ✅ **Stops when unsure** if anything looks off, it pauses and asks for help

## How it works

1. **Look and confirm**  
   The camera finds the tray, cuvette and analyser markers, then checks the scene matches what is expected.

2. **Plan the move**  
   The system picks a safe, efficient path for the next step, like open lid or place cuvette.

3. **Do the action**  
   The arm carries out the step and confirms the result. If misaligned or a spill is detected, it stops.

## Why it matters

- Frees people from repetitive handling  
- Reduces small placement errors that waste samples  
- Shows a clear path to broader lab tasks in the future

## What you can see in this repo

- 📄 **Overview docs** and visuals that explain the workflow and design  
- 🖼️ **Diagrams and assets** for posters and the project website  
- 🧪 **Demo sequences** that outline the step order for a typical run

> Looking for installation or developer detail? Those belong in separate docs to keep this page friendly.

## Current status

- [x] Core vision with markers identifies targets
- [x] Step-by-step sequences run in simulation first
- [x] Arm picks, places, and closes the lid in a controlled setup
- [ ] Extended testing and polish for exhibition scenarios

## Acknowledgements

Built by a small team of mechatronics students. Thanks to our supervisors and lab staff for space, feedback and safety guidance.
