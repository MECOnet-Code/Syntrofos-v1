# Overview

**Syntrofos 1.1** is a system that utilizes temperature, heart rate (HR), and respiratory rate (RR) sensors to monitor vital signs and provide visual indicators through LEDs.

The code was written for Arduino Nano but it can be applied to other processors by editing their timer function.

Syntrofos is originally intended for detecting COVID-19 symptoms.
It can monitor three parameters:
- Heart rate (PPG)
- Respiratory rate
- Body temperature

If any of these parameters is out of normal range it can point to COVID-19 infection.

![Syntrofos-v1 device](https://github.com/MECOnet-Code/Syntrofos-v1/assets/154332242/68ebc768-dff0-4a0c-833a-de651485bbe1)

## Interface
Syntrofos features 3 dual-color LEDs for each monitored signal. The simplest way to indicate that a certain signal is OK is to light it's green LED, red LED will light up otherwise.

LED signaling is divided in two parts:
- 10 seconds: heart rate (**HR**) LED blinks at the rhythm of heart beats and respiratory rate (**RR**) LED blinks at the rhythm of breathing
- 4 seconds: all three LEDs light up according to their correspondive signal: green if good or red if bad

Additionally, if at least one red LED was lighted - a BLE beacon will turn on to signal an alarm to a remote controller.

Since the project was  built fot Arduino nano, there is also serial communication available for monitoring live signals and numerical results in serial monitors.
![syntrofos-v1-graphs](https://github.com/MECOnet-Code/Syntrofos-v1/assets/154332242/c67c445b-91ee-4732-8f8f-eb04a6fd75d8)

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

### Beacon Indicator

-   **Beacon Indicator Pin:** Connected to pin 6.

## Parameters

-   **LEDinterval:** Duration of LED signal in milliseconds (ms).
-   **Sampling Frequency (fs):** 32 Hz.
-   **Number of Pulses to count rate (NUM_PULSES):** 4. Smaller number can make faster results, but less precision.
-   **Filter Coefficients (α and β):** Calculated based on specified cutoff frequencies.

## Filter Design

-   **Low Pass (LP) Filters:**
    -   Heart Rate (HR): Cutoff frequency (fc) = 1.0 Hz.
    -   Respiratory Rate (RR): Cutoff frequency (fc) = 1.0 Hz.
-   **High Pass (HP) Filters:**
    -   Heart Rate (HR): Cutoff frequency (fc) = 0.5 Hz.
    -   Respiratory Rate (RR): Cutoff frequency (fc) = 0.4 Hz.

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
-   The LEDs temporarily turn off between status updates to conserve power.
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

