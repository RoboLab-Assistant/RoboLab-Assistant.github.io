<p align="center">
  <img src="docs/logos/RoboLab Assistant - Square (Dark).svg" alt="RoboLab Assistant logo" width="140">
</p>

<p align="center">
  <!-- Inline SVG wordmark so we can colour just “Lab” and request Inter Medium -->
  <svg xmlns="http://www.w3.org/2000/svg" role="img" aria-label="RoboLab Assistant" width="520" height="60">
    <style>
      text { font-family: Inter, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif; font-weight: 500; }
    </style>
    <text x="0" y="45" font-size="40" fill="#ffffff">Robo</text>
    <text x="110" y="45" font-size="40" fill="#0fa4c0">Lab</text>
    <text x="190" y="45" font-size="40" fill="#ffffff"> Assistant</text>
  </svg>
</p>

<p align="center">
  A compact robotic helper that moves test tubes (cuvettes) safely between a tray and a lab analyser.
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

Built by a small team of mechatronics students. Thanks to our mentors and lab staff for space, feedback and safety guidance.

---

<p align="center">
  <sub>Wordmark uses Inter Medium. “Lab” is coloured <code>#0fa4c0</code> to match the project theme.</sub>
</p>
