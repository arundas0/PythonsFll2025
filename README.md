# FLL 2025 Missions — Kid-Friendly Guide

Welcome! This folder contains Python code used by FLL (FIRST LEGO League) teams to run missions for the 2025 challenge. The files here are designed so a kid team can pick them up, run them, and change them to make the robot do new things.

Who this is for
- Kids, teammates, and coaches learning how to run robot missions.
- Parents or mentors helping with setup and safety.

What’s in this folder
- `FLL2025Missions.py` — main mission routines and helpers.
- `FllPython2025_01022026.py` — an experimental mission script.
- `test_gyro_turn.py` — a small test that checks turning using a gyro.
- `requirements.txt` — lists the main dependency used here.

Quick idea: how to use these files
1. Read the code with your team and decide which mission to try first.
2. Make a copy of the script before you change it, so you can always go back.
3. Change small things (speed, time, distance) and try again.

Requirements
- This project uses Pybricks (the Python framework for LEGO hubs).
- See `requirements.txt` for the main dependency name.

How to set up Pybricks (kid-friendly steps)
1. Ask an adult to help — they should be nearby when you first connect hardware.
2. Use the Pybricks Code web app (recommended for beginners):
   - Open the Pybricks Code website in a browser.
   - Follow the instructions there to install the Pybricks firmware on your LEGO hub (this is a one-time step).
3. If you prefer using a computer and a USB cable, you can install the helper tools:

   ```bash
   pip install pybricksdev
   ```

   - `pybricksdev` helps you upload Python files to the hub from your PC.
   - To copy a file to the hub from the command line (example):

   ```bash
   pybricksdev copy FLL2025Missions.py
   ```

   (Tip: using Pybricks Code web app is easier for beginners; `pybricksdev` is for more advanced control.)

Running the code
- Using Pybricks Code: upload the chosen `.py` file to your hub and run it from the hub menu.
- Using `pybricksdev`: upload the script and then run it on the hub.
- If you do not have a hub, you can still read and edit the scripts on your computer to learn programming.

Simple safety rules
- Keep fingers away from wheels, gears, and moving parts.
- Have one person watch the robot while someone else runs the code.
- If anything unexpected happens, stop the robot right away and unplug it if needed.

How to adapt missions (easy steps)
- Change numbers slowly: reduce speed or distance to test.
- Add `print()` statements to see what the code is doing (helpful for debugging).
- Save your changes in a new file like `my_mission.py` so your team doesn't lose the original.

Troubleshooting tips
- If the hub doesn’t respond, check the battery or charge it.
- Make sure the hub is connected and the right program is uploaded.
- If an import fails, double-check `requirements.txt` and ask an adult to help install the tools.

Have fun building and coding — good luck at FLL 2025!
