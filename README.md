# Digital 0 to 10 V Power Supply with Deadbeat Control

Closed-loop variable voltage source (0 to 10 V) controlled by a discretized deadbeat controller running on an ESP32. Individual project, with firmware and a MATLAB analysis package.

![Setup](images/setup.jpg)

## Overview

[TODO: 2 to 3 sentences from the report introduction: goal and why deadbeat control.]

## Hardware

- ESP32-WROOM-32 on a PAT-DAELN v2.1 development board, used in a UTFPR microcontrollers course and reused for this project
- ADS1015 ADC for output voltage feedback
- SSD1306 OLED display
- PWM output on GPIO18 through the LC-LM358-PWM2V module, which converts the PWM signal into an analog voltage

## Control design

Discretized deadbeat controller with sampling time Ts = 250 ms, based on the identified plant gain (K) and time constant (tau). Final parameters:

| Parameter | Value |
|---|---|
| K | 1.01 |
| tau | 30 ms |
| Ts | 250 ms |
| KI_TRIM | 0.15 |
| Bias limit | +/- 2 V |
| Error dead zone | 0.02 V |
| Trim error limit | 0.5 V |

[TODO from the report: one sentence on what the trim term does and one on the bias term.]

## Results

[TODO from the report: rise time, settling time, steady-state error (under 2%), step response plot, Bode and Nyquist plots.]

## MATLAB analysis

The analysis package computes the step response, Bode and Nyquist plots and the rise time, using zero-order hold interpolation. Data are captured from the Arduino IDE serial monitor and imported into MATLAB, because opening the serial port from MATLAB toggles DTR/RTS and resets the ESP32.

## Repository structure

```
firmware/   ESP32 source code
analysis/   MATLAB scripts and sample data
images/     photos and result plots
```

## How to reproduce

[TODO: Arduino IDE version, board selection, required libraries, script names and run order.]

## Author

Felipe Martins, Mechatronics Engineering student at UTFPR, double degree at ENSTA Bretagne.
[LinkedIn](https://www.linkedin.com/in/felipe-m-k-m-martins)
