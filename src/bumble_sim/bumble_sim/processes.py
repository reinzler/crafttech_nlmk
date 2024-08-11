
#!/usr/bin/env python3

# Import the subprocess and time modules
import subprocess
import time

# List of commands to run
commands = [
    # Run the Micro XRCE-DDS Agent
    "MicroXRCEAgent udp4 -p 8888",

    # Run the PX4 SITL simulation
    # "cd ~/PX4-Autopilot && make px4_sitl gazebo-classic_r1_rover",

    # Run QGroundControl
    "QGroundControl.AppImage"
]

# Loop through each command in the list

for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])

    # Pause between each command
    time.sleep(1)

# Откройте первый терминатор
# subprocess.run(["terminator", "-x", "bash", "-c", commands[0] + "; exec bash"])
#
# for i in range(1, len(commands)):
#     # Каждую следующую команду запускаем в новом сплите
#     subprocess.run(["terminator", "--new-tab", "-x", "bash", "-c", commands[i] + "; exec bash"])
#     time.sleep(1)