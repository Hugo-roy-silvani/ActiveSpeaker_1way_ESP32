# ActiveSpeaker_1way_ESP32

*A mono active speaker system built around an ESP32, featuring a modular real-time DSP pipeline and Bluetooth control. This project explores embedded audio processing on low-cost hardware and serves as a foundation for more advanced DSP experiments.*
![IMG_6781](https://github.com/user-attachments/assets/4998e6bc-37f2-4178-8a68-510827150cec)

---

## Features

- Realtime audio pipeline (128-sample processing blocks)
- I2S audio output
- DSP modules:
  - IIR biquad filtering
  - Basic EQ
  - Gain control
  - Optional limiter/expander (depending on CPU load)
- Bluetooth Low Energy (BLE) parameter control
- Modular architecture for easy extension

---

## DSP Pipeline Overview

Bleutooth → Ring Buffer → IIR Filters → EQ/Gain → (Optional Dynamics) → I2S Output

Processing is done in floating-point using ESP-DSP or custom routines, depending on the module.

---

## Hardware Used

- **ESP32 DevKitC / ESP32-WROOM module**  
- **I2S amplifier** — MAX98357A  
- **speaker** (FRS5X - 8 Ohm)  
- Optional: USB-serial for debugging & parameter monitoring  

This hardware setup demonstrates a complete embedded audio path from capture → DSP → amplification.

---

## Build & Compilation Requirements

### **Prerequisites**
- **ESP-IDF ≥ v5.0** (recommended)
- Python 3.10+
- USB-serial driver for ESP32
  
~~~bash
#Create projet
idf.py create-project "project name"

# Configure target and environment
idf.py set-target esp32

# Build
idf.py build

# Flash to device
idf.py flash

# Monitor serial output (Do not use if using GUI)
idf.py monitor
~~~

## Structure 

ActiveSpeaker_1way_ESP32/  
│── CMakeLists.txt  
│── sdkconfig  
│── main/  
│   ├── app_audio.h/.c       # I2S drivers, buffering  
│   ├── app_bt.h/.c            # bluetooth  
│   ├── app_dsp.h/.c          # DSP  
│   ├── board_config.h  
│   └── main.c  
└── README.md  
