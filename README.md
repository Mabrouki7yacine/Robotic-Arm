This project simulates and controls a 2-joint robotic arm in real-time using a graphical interface and TCP communication. The Python-based GUI enables interactive control of arm positioning via inverse kinematics, and communicates the calculated joint angles to an ESP32 microcontroller over a Wi-Fi TCP connection. The ESP32 receives the angles and drives two servo motors to achieve the desired arm pose.

Features
2D Simulation with Inverse Kinematics (Python + Matplotlib)
Real-time GUI Input using Tkinter for target position control
Smooth Arm Animation with easing transitions
TCP Server on PC for sending angles
TCP Client on ESP32 for receiving commands
Servo Control on ESP32 using FreeRTOS tasks
Reachability Handling for out-of-range targets

Tech Stack
Python (matplotlib, tkinter, socket)
C (ESP-IDF for ESP32)
Wi-Fi TCP communication
Servo control logic with safety limits

Applications
Educational robotics
Inverse kinematics experimentation
Wireless robotic arm prototypes
Embedded systems projects
