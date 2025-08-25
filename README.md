# Autonomous Drone GPS Navigation with Camera and QR Code Detection

This repository contains the complete software stack for an autonomous drone designed to navigate to a specific GPS location, perform a systematic aerial search, and use computer vision to detect and decode QR codes.We had issues with mounting the Pi camera on the drone.

---

## 📝 Project Overview

The primary goal of this project is to create an unmanned aerial system capable of autonomously finding and extracting data from QR codes within a designated geographical area. The system integrates GPS-based navigation for long-range travel with a vision-based detection system for precise, close-range target identification. The mission is executed without manual intervention after launch.

## ⚙️ Core Functionality

* **Autonomous Navigation:** The drone takes off and flies to a predefined set of GPS coordinates using DroneKit for high-level flight control.
* **Search Protocol:** Upon arrival at the target location, the drone initiates a "descend-and-scan" routine, incrementally lowering its altitude while hovering to ensure optimal conditions for image capture.
* **AI-Based QR Code Detection:** At each altitude step, the system captures high-resolution images which are processed by a custom-trained YOLOv5 model to accurately locate QR codes within the camera's field of view.
* **Data Extraction:** Once a QR code is detected, its contents are decoded using the `pyzbar` library. The system can process and save both plain text data and base64-encoded images found within the QR codes.
* **Optimized Imaging:** The camera is configured to capture sharp, 12-megapixel (4056x3040) images with settings specifically tuned for clarity, such as increased sharpness and contrast, to improve QR code recognition.
* **Safety and Failsafes:** The flight script includes logic to detect if the drone overshoots its target destination and will trigger a Return-to-Launch (RTL) if it deviates from its path.

---

## 🛠️ System Architecture

### Hardware

* **Drone Platform:** A multi-rotor UAV (e.g., quadcopter).
* **Flight Controller:** A Pixhawk or similar controller running ArduPilot/PX4 firmware.
* **Companion Computer:** A Raspberry Pi (or equivalent single-board computer) connected to the flight controller via a serial (UART) connection.
* **Camera:** A high-resolution Raspberry Pi Camera Module.

### Software

* **Flight Control:** `dronekit`
* **Computer Vision:** `torch`, `opencv-python`
* **QR Decoding:** `pyzbar`
* **Camera Interface:** `picamera2`
* **GPS Utilities:** `geopy`

---

## 🧑‍💻 Code Breakdown

The project is segmented into three key Python scripts:

1.  **`gps.py`**: This is the central mission control script. It manages the connection to the flight controller, handles all flight stages (takeoff, navigation, landing), and orchestrates the search-and-scan sequence by invoking the image capture script at specific altitudes.

2.  **`cam.py`**: This script is responsible for all camera operations. It configures the camera for high-resolution still capture and applies optimized settings for exposure, gain, and sharpness to ensure clear images for the vision system.

3.  **`qr.py`**: This script contains the computer vision logic. It loads a pre-trained YOLOv5 model to detect QR codes in an image, draws bounding boxes, and uses `pyzbar` to extract the embedded data. It also calculates the QR code's offset from the image center in meters, providing data for precise positioning.

---


## ▶️ Execution

To launch the autonomous mission, run the main script from the terminal:

~~~
python3 gps.py
~~~

Testing Images:

<img width="1600" height="900" alt="image" src="https://github.com/user-attachments/assets/e86f100d-eeab-41dd-a1dc-4e5bf0292638" />

<img width="1600" height="900" alt="image" src="https://github.com/user-attachments/assets/eae5ac25-fa90-44dd-83e7-6e409d170644" />

<p align="center">
  <img src="https://github.com/user-attachments/assets/20bf21ed-99da-4ea8-aea4-b215ed173baf" alt="image" width="438" height="425">
</p>
