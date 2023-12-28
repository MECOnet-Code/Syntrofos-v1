# Syntrofos v1 time domain analysis

This is a time domain processing code to obtain heart rate and respiratory rate for Syntrofos device.
The code was written for Arduino Nano but it can be applied to other processors by editing their timer function.

# What is Syntrofos

Syntrofos is a medical device intended for detecting COVID-19 symptoms.
It can monitor three parameters:
- Heart rate (PPG)
- Respiratory rate
- Body temperature

If any of these parameters is out of usual range it can point to COVID-19 infection.
## Interface
Syntrofos features 3 dual-color LEDs for each monitored signal. The simplest way to indicate that a certain signal is OK is to light it's green LED, red LED will light up otherwise.

LED signaling is divided in two parts:
- 10 seconds: heart rate (**HR**) LED blinks at the rhythm of heart beats and respiratory rate (**RR**) LED blinks at the rhythm of breathing
- 4 seconds: all three LEDs light up according to their correspondive signal: green if good or red if bad

Additionally, if at least one red LED was lighted - a BLE beacon will turn on to signal an alarm to a remote controller.

Since the project was  built fot Arduino nano, there is also serial communication available for watching live signals and numerical results in serial monitors which can also support logging for detailed analysis if needed.
