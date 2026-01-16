# **SFX:** A PROS Library  
  
A lightweight, high-performance library for robotics simulations, path handling, and visual debugging in PROS.  
  
This project features three main subsystems:  
1. The `logger`  
2. The `screen` utilities  
3. The `motorCheck` functions  

# The Logger
The **Logger** is a multi-threaded telemetry system designed to record robot performance data to the V5 Brain's SD card and the debug terminal. It includes integrated hardware safety checks and support for external graphing tools.

### Key Features

* **Dual-Stream Logging**: Can simultaneously writes to the terminal (stdout) and the SD card.
* **Smart File Management**:
    * Automatic file naming using the V5 RTC (Real Time Clock) in `YYYY-MM-DD` format.
    * **Auto-Save Protection**: Periodically closes and re-opens the log file to prevent data loss if the battery is pulled unexpectedly.
* **Startup Handshake**: Optional `waitForSTDin` mode pauses execution until a `Y` key is received in the terminal, ensuring early initialization logs are not missed.
* **JerryIO Support**: A dedicated CSV mode outputs pose (`X, Y, Theta`) and velocity data formatted specifically for the JerryForge visualization tool, provided in  
> **[tools/JerryForge/](tools/JerryForge/README.md)**  

# Screen Utilities
The **screen** namespace contains several 