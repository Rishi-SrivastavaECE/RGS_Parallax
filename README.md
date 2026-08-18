# RGS_Parallax
Hackathon Parallax
# Contributors-
- Rishi Srivastava 24BEC1257
- Ayush Gupta      24BEC1140
- A Ayush          24BEC1299
- Samarth M        24BEC1242

# Runway Alignment Guidance & Aircraft Response Analysis System

A distributed, sensor-fusion-based prototype for runway approach guidance and aircraft response analysis.

The system combines **three ESP32 ground nodes, RSSI-based lateral alignment, ultrasonic sensing, an MPU6050-equipped aircraft rig, and a Raspberry Pi analysis station** to demonstrate an intelligent approach-monitoring system.

## Overview

Traditional Instrument Landing Systems provide pilots with lateral and vertical guidance using dedicated radio-navigation infrastructure. This project explores a low-cost experimental architecture that reproduces some of the **guidance, redundancy, monitoring, and response-analysis concepts** using readily available embedded hardware.

The system is divided into two major layers:

### Ground Guidance Layer

Three ESP32 nodes are arranged along the runway approach:

- **Left ESP32** — measures aircraft RSSI
- **Centre ESP32** — measures aircraft RSSI and performs alignment analysis
- **Right ESP32** — measures aircraft RSSI

The Centre node compares the received signal strengths:

```text
Left RSSI     Centre RSSI     Right RSSI
     \             |             /
      \            |            /
       └────── Alignment ───────┘
