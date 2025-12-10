# ActiveSpeaker_1way_ESP32

*A mono active speaker system built around an ESP32, featuring a modular real-time DSP pipeline and Bluetooth control. This project explores embedded audio processing on low-cost hardware and serves as a foundation for more advanced DSP experiments.*

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
- Python 3.8+
- USB-serial driver for ESP32 
