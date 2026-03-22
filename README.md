<div align="center">
  <h1>🚁 Drone Flight Controller RTOS</h1>
  <p>A custom Real-Time Operating System flight controller for quadcopters built on FreeRTOS.</p>

  [![Buy Me A Coffee](https://img.shields.io/badge/Buy%20Me%20A%20Coffee-Donate-yellow.svg)](https://www.paypal.com/donate/?business=freelacning8518@gmail.com)
</div>

## Overview
This repository contains a highly optimized flight controller firmware designed specifically for STM32 microcontrollers. It features:
* **FreeRTOS Integration:** Guarantees hard real-time execution of the 1kHz flight control loop.
* **Madgwick Sensor Fusion:** A lightweight AHRS algorithm that computes 3D orientation (roll, pitch, yaw) from MPU9250 IMU data.
* **Cascaded PID Controller:** Dual-loop stabilization (angle outer loop + rate inner loop).

## Support & Contact
If you found this useful, consider buying me a coffee to support further open-source hardware work!

Interested in custom embedded systems, IoT, or edge AI development? Let's connect!
* **Upwork:** [Ahmed Qandel](https://www.upwork.com/freelancers/ahmedqandel)
* **Portfolio:** [ahmedqandel.com](https://ahmedqandel.com)
