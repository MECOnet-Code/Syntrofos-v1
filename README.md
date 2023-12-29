# Overview

**Syntrofos 1.1** is an embedded system designed for time-domain processing, integrating temperature, heart rate (HR), and respiratory rate (RR) sensors. The system employs digital signal processing techniques to analyze sensor data and provides visual feedback through LEDs.

This README provides an in-depth understanding of the code's structure, functionality, and key components.

For more information, a paper **SYNTROFOS: A Wearable Device for Vital Sign Monitoring, Hardware and Signal Processing Aspects** can be found on [ResearchGate](https://www.researchgate.net/publication/371888548_SYNTROFOS_A_Wearable_Device_for_Vital_Sign_Monitoring_Hardware_and_Signal_Processing_Aspects).

The general code was written for Arduino Nano but it can be applied to other processors as well by editing their timer function. The ESP32 is only used to detect BLE beacon which acts as an alarm signal and make beep sounds while it is on. Any other BLE device or module can be used instead.

Syntrofos is originally intended to be used for detecting COVID-19 symptoms.
It can monitor three parameters:
- Heart rate (PPG)
- Respiratory rate
- Body temperature

If any of these parameters is out of normal range it can point to COVID-19 infection.

![Syntrofos-v1 device](https://github.com/MECOnet-Code/Syntrofos-v1/assets/154332242/68ebc768-dff0-4a0c-833a-de651485bbe1 "Syntrofos v1 prototype")

## Interface
Syntrofos features 3 dual-color LEDs for each monitored signal. The simplest way to indicate that a certain signal is OK is to light it's green LED, red LED will light up otherwise.

LED signaling is divided in two parts:
1. heart rate (**HR**) LED blinks at the rhythm of heart beats and respiratory rate (**RR**) LED blinks at the rhythm of breathing
2. all three LEDs light up according to their correspondive signal: green if good or red if bad

Additionally, if at least one red LED was lighted - a BLE beacon will turn on to signal an alarm to a remote controller.

There is also serial communication available for monitoring live signals and numerical results in serial monitor.
![syntrofos-v1-graphs](https://github.com/MECOnet-Code/Syntrofos-v1/assets/154332242/c67c445b-91ee-4732-8f8f-eb04a6fd75d8 "Heart and respiratory plot")

## Components

### Sensors

- **Temperature Sensor:** Utilizes a DallasTemperature sensor connected to pin 2.
- **Heart Rate Sensor:** Connected through operational amplifier to analog pin A1.
- **Respiratory Rate Sensor:** Connected through operational amplifier to analog pin A2.

### LEDs

-   **Heart Rate LEDs:**
    -   Green LED (HR normal) on pin 12.
    -   Red LED (HR abnormal) on pin 11.
-   **Respiratory Rate LEDs:**
    -   Green LED (RR normal) on pin 10.
    -   Red LED (RR abnormal) on pin 9.
-   **Temperature LEDs:**
    -   Green LED (normal) on pin 8.
    -   Red LED (abnormal) on pin 7.

### BLE Beacon

-   **Beacon device Pin:** Connected to pin 6.

This was the common part for both time and frequency domains. The differences between two are described next.

# Time domain analysis

## Filter Design

-   **Low Pass (LP) Filters:**
    -   Heart Rate (HR): Cutoff frequency (fc) = 1.0 Hz.
    -   Respiratory Rate (RR): Cutoff frequency (fc) = 1.0 Hz.
-   **High Pass (HP) Filters:**
    -   Heart Rate (HR): Cutoff frequency (fc) = 0.5 Hz.
    -   Respiratory Rate (RR): Cutoff frequency (fc) = 0.4 Hz.

## Parameters

-   **LEDinterval:** Duration of LED signal in milliseconds (ms).
-   **Sampling Frequency (fs):** 32 Hz.
-   **Number of Pulses to count rate (NUM_PULSES):** 4. Smaller number can make faster results, but less precision.
-   **Filter Coefficients (α and β):** Calculated based on specified cutoff frequencies.

## System Operation

1.  **Initialization:**
    -   Sets up pins, initializes filters, and configures timer interrupts.
2.  **Main Loop:**
    -   Samples HR and RR signals at 32 Hz, and temperature at 6 Hz (every 10 seconds)
    -   Applies LP and HP filters to extract pulse signals.
    -   Detects rising edges to count pulses and calculate rates.
3.  **LED Indicators:**
    -   Heart Rate LEDs indicate normal/green or abnormal/red status.
    -   Respiratory Rate LEDs indicate normal/green or abnormal/red status.
    -   Temperature LEDs indicate normal/green or abnormal/red status.
4.  **Beacon Indicator:**
    -   Turns on/off based on abnormal sensor readings.
    -   Ensures a 3-second delay for stable switching operation.
5.  **Temperature Monitoring:**
    -   Checks for abnormal temperature readings or sensor disconnection.
6.  **Serial Output:**
    -   Optionally outputs temperature, HR, RR, and filtered signal values to Serial.

## Notes

-   The system performs continuous monitoring and updates LED status every 10 seconds.
-   The LEDs blink between status updates to conserve power. For example, while breathing out the green RR LED will blink fast instead of light constantly until signal drops below treshold.
-   Beacon indicator status reflects abnormal sensor readings.

## Functions

-   **filter_hr:** Applies LP and HP filters to the HR signal.
-   **filter_rr:** Applies LP and HP filters to the RR signal.
-   **beaconPowerLogic:** Manages beacon indicator power based on alarm status.
-   **LEDstatus:** Sets the color of LEDs based on HR, RR, and temperature readings.
-   **findRate:** Calculates HR or RR based on the number of pulses detected.
-   **isRisingEdge:** Checks for a rising edge in binary pulse signals.
-   **checkT:** Checks if the pulse period is within the expected range.
-   **printToSerial:** Outputs sensor readings and filtered signals to Serial.

# Frequency domain analysis

## **Main Loop:**
    
-   The `loop` function continuously processes samples from RR and HR sensors in real-time.
-   Each sample undergoes high-pass filtering to extract relevant signal components. Because FFT analysis is more noise tolerant, low pass filter is omitted for saving memory.
    -   Heart Rate (HR): Cutoff frequency (fc) = 2.5 Hz.
    -   Respiratory Rate (RR): Cutoff frequency (fc) = 0.4 Hz.
-   LEDs indicate the state of the filtered signals, providing visual feedback.
    
## **Signal Analysis:**
    
-   FFT analysis is employed for frequency domain processing to identify dominant spectral components.
-   The breathing rate (RRR) and heart rate (RHR) are calculated based on the FFT results.
    
## **Error Handling:**
    
-   Error codes are generated to classify the quality of the signals.
-   LED indicators respond to error codes, signaling potential issues in signal strength or reliability.
    
## **Temperature Monitoring:**
    
-   Temperature readings are obtained from a sensor and used as an additional health parameter.
-   LEDs indicate whether the temperature is within a normal range or potentially alarming.
    
## **Beacon Functionality:**
    
-   A beacon mechanism is integrated to signal out of safe range results or low signal conditions.
-   Beacon power logic manages the activation and deactivation of the beacon based on signal quality.
    
## **Functions:**
    
-   Various functions handle specific tasks, such as resetting variables, calculating error codes, managing LED status, and implementing beacon power logic.
-   High-pass filtering functions are used to process RR and HR signals.
    
## **Interrupts:**
    
-   Timer 1 interrupt is utilized for controlling the sampling frequency, ensuring consistent and periodic processing.
    
## **Serial Output:**

-   After each cycle, heart and respiratory spectrums are printed and at the end the results, including breathing rate, heart rate, and temperature are printed for monitoring and debugging.


