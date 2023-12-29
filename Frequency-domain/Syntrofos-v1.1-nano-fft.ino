// Syntrofos 1.1 frequency domain processing

#include "fix_fft.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#define RATIO_RR 0.3      // good signal RR ratio, determined experimentally
#define RATIO_HR 0.19     // good signal HR ratio, determined experimentally
#define LEDinterval 5000  // LED signalling interval in ms

#define ONE_WIRE_BUS 2    // temperature sensor pin
#define RRpin A2          // RR sensor pin
#define HRpin A1          // HR sensor pin
#define LEDt_red 7        // teperature red LED pin
#define LEDt_gr 8         // temperature green LED pin
#define LEDrr_red 9       // RR red LED pin
#define LEDrr_gr 10       // RR green LED pin
#define LEDhr_red 11      // HR red LED pin
#define LEDhr_gr 12       // HR green LED pin
#define BEACONpin 6       // beacon pin

int intr_count = 0;                 // interrupts counter
int Tms;                            // sampling period in ms
bool canSample = false;             // sampliing inidicator
int sample_count = 0;               // sample counter
bool firstSpecutrumDone = false;    // the first spectrum indicator which is discarded because of noise
float PeakRR, PeakHR;               // dominant spectrum components
int maxSP_rr, maxSP_hr = 0;         // spectrum maximal values
int err_rr, err_hr = 30;            // error code, 0: normal rhythm, 1: slower, 2: faster, -1: low signal
int sumS_rr, sumS_hr = 0;           // spectrum sums
int sum3_rr, sum3_hr = 0;           // 3 dominant spectrum elements sums
float RRR, RHR = 0.0;               // real respiratory rate and heart rate

const byte FFT_N = 128;             // buffer size
char im_rr[FFT_N], data_rr[FFT_N];  // imaginary and real arrays for fft
char im_hr[FFT_N], data_hr[FFT_N];
int dat_rr, dat_hr = 0;             // fft vector intensity variables
int iHrRr = 0;                      // iterator for fft arrays
int8_t maxiin_rr, maxiin_hr = 0;    // max values indexes for fft
int maxi_rr, maxi_hr = 0;           // max values for fft
const byte ZP = 128;                // zero padding possibility if set for example to 64
const byte fs = 8;                  // sampling frequency

// Filter variables
float beta_rr, beta_hr = 0.0;       // HP filter 1st order
float y_hp_rr, y_hp_hr = 0.0;
float x_hp_rr, x_hp_hr = 0.0;

float Tcel = 0.0; // temperature variable

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


void setup()
{
  // Filter variables
  const float fc_hp_rr = 0.4;
  const float fc_hp_hr = 2.5;

  // declare LED pins as output
  pinMode(LEDt_gr, OUTPUT);
  pinMode(LEDt_red, OUTPUT);
  pinMode(LEDrr_gr, OUTPUT);
  pinMode(LEDrr_red, OUTPUT);
  pinMode(LEDhr_gr, OUTPUT);
  pinMode(LEDhr_red, OUTPUT);
  pinMode(BEACONpin, OUTPUT);

  analogReference(INTERNAL);               // a built-in reference, equal to 1.1 volts on the ATmega168 or ATmega328P
  Serial.begin(115200);
  Serial.println("Syntrofos v1.1");

  sensors.begin();
  sensors.setWaitForConversion(false);     // if set to false then it is not blocking, but then 750ms or more must be waited before next call
  cli();                                   // disable interrupts during setup
  LEDsOff();                               // set LEDs initially to off
  beta_rr = calculate_beta(fc_hp_rr, fs);  // filter coefficients
  beta_hr = calculate_beta(fc_hp_hr, fs);
  Tms = (int) (1000 / fs);                 // sampling period
  delay(500);
  comp_match();
  sei();                          // re-enable interrupts
} // setup


uint8_t D = 25;                   // variable for hysteresis effect when blinking LEDs
int average_rr, average_hr = 0;   // average signal values
int sampleRR, sampleHR;           // sample variables
bool alarmSignal = false;                   // if any of 3 monitored signals is out of safe range
bool beaconStatus, beaconPowering = false;  // false: off, true: on
int beaconPower_count = 0;                  // to count beacon powering duration

void loop()
{
  int sum_rr, sum_hr = 0;   // sums to calculate average values
  int yo_hp_rr, yo_hp_hr;   // filter outputs
  float P_left_rr, P_left_hr, P_rigth_rr, P_right_hr = 0.0;  // left and right spectral components around the maximum
  float mi_rr, mi_hr = 0.0;                                  // rhythm correction coefficients

  if (canSample == true)
  {
    cli();
    ReadInternal(&sampleRR, &sampleHR); // take samples
    sample_count++;                     // count samples

    // filter rr signal
    yo_hp_rr = high_pass_rr(beta_rr, float(sampleRR));
    yo_hp_rr = limit(yo_hp_rr, -127, 127);
    // filter hr signal
    yo_hp_hr = high_pass_hr(beta_hr, float(sampleHR));
    yo_hp_hr = limit(yo_hp_hr, -127, 127);

    // blink RR and HR LEDs at the pace of their signals
    if (yo_hp_rr < -(average_rr + D))
      digitalWrite(LEDrr_gr, LOW);
    else if (yo_hp_rr > -(average_rr + D / 2))
      digitalWrite(LEDrr_gr, HIGH);
    if (yo_hp_hr > average_rr + D)
      digitalWrite(LEDhr_gr, LOW);
    else if (yo_hp_hr < average_rr + D / 2)
      digitalWrite(LEDhr_gr, HIGH);

    data_rr[iHrRr] = (char)yo_hp_rr;
    data_hr[iHrRr] = (char)yo_hp_hr;
    iHrRr++;
    sum_rr += yo_hp_rr;
    sum_hr += yo_hp_hr;
    if (iHrRr % 32 == 0) {                 // calculate average value after every 32 samples
      average_rr = (int) sum_rr / 32;
      average_hr = (int) sum_hr / 32;
      sum_rr = 0;
      sum_hr = 0;
    }

    // do FFT
    if (iHrRr > ZP)
    {
      // fill imaginary
      for (int i = 0; i < FFT_N; i++) {
        im_rr[i] = 0;
        im_hr[i] = 0;
      }

      fix_fft(data_rr, im_rr, 6, 0); // FFT functions
      fix_fft(data_hr, im_hr, 6, 0);

      maxi_rr = 0;
      maxi_hr = 0;
      maxiin_rr = 0;
      maxiin_hr = 0;
      PeakRR = 0;
      PeakHR = 0;

      sensors.requestTemperatures();      // send the command to get temperature readings

      for (int i = 1; i < FFT_N / 2; i++) // full spectrum rr; index 0 is dc component and is not considered
      {
        dat_rr = (int8_t(data_rr[i]) * int8_t(data_rr[i])) + (int8_t(im_rr[i]) * int8_t(im_rr[i]));
        sumS_rr += dat_rr;                // add all elements to sum
        Serial.print((int)dat_rr);
        Serial.print("\r\n");

        if (dat_rr > maxi_rr) {   // find max in rr spectrum
          maxi_rr = dat_rr;
          maxiin_rr = i;
          PeakRR = dat_rr;
        }
      }

      for (int i = 1; i < FFT_N / 2; i++) // full spectrum hr; index 0 is dc component and is not considered
      {
        dat_hr = (int8_t(data_hr[i]) * int8_t(data_hr[i])) + (int8_t(im_hr[i]) * int8_t(im_hr[i]));
        sumS_hr += dat_hr;
        Serial.print((int)dat_hr);
        Serial.print("\r\n");

        if (dat_hr > maxi_hr) {   // find max in hr spectrum
          maxi_hr = dat_hr;
          maxiin_hr = i;
          PeakHR = dat_hr;
        }
      }

      maxSP_rr = (int) PeakRR;    // max
      sum3_rr += PeakRR;          // sum of three peak elements
      maxSP_hr = (int) PeakHR;
      sum3_hr += PeakHR;

      // calculate bpms based on fft analysis
      RRR = findRate(maxiin_rr);
      RHR = findRate(maxiin_hr);

      // find left and right amplitudes around peak value
      P_rigth_rr = (float) (int8_t(data_rr[maxiin_rr + 1]) * int8_t(data_rr[maxiin_rr + 1])) + (int8_t(im_rr[maxiin_rr + 1]) * int8_t(im_rr[maxiin_rr + 1]));
      sum3_rr += P_rigth_rr;
      P_left_rr = (float) (int8_t(data_rr[maxiin_rr - 1]) * int8_t(data_rr[maxiin_rr - 1])) + (int8_t(im_rr[maxiin_rr - 1]) * int8_t(im_rr[maxiin_rr - 1]));
      sum3_rr += P_left_rr;

      P_right_hr = (float) (int8_t(data_hr[maxiin_hr + 1]) * int8_t(data_hr[maxiin_hr + 1])) + (int8_t(im_hr[maxiin_hr + 1]) * int8_t(im_hr[maxiin_hr + 1]));
      sum3_hr += P_right_hr;
      P_right_hr = (float) (int8_t(data_hr[maxiin_hr + 2]) * int8_t(data_hr[maxiin_hr + 2])) + (int8_t(im_hr[maxiin_hr + 2]) * int8_t(im_hr[maxiin_hr + 2]));
      sum3_hr += P_right_hr;
      P_left_hr = (float) (int8_t(data_hr[maxiin_hr - 1]) * int8_t(data_hr[maxiin_hr - 1])) + (int8_t(im_rr[maxiin_hr - 1]) * int8_t(im_hr[maxiin_hr - 1]));
      sum3_hr += P_left_hr;
      P_left_hr = (float) (int8_t(data_hr[maxiin_hr - 2]) * int8_t(data_hr[maxiin_hr - 2])) + (int8_t(im_rr[maxiin_hr - 2]) * int8_t(im_hr[maxiin_hr - 2]));
      sum3_hr += P_left_hr;

      // result correction
      mi_rr = -(P_rigth_rr - P_left_rr) / ((2.0 * PeakRR) - P_left_rr - P_rigth_rr);
      RRR = RRR - mi_rr;
      mi_hr = -(P_right_hr - P_left_hr) / ((2.0 * PeakHR) - P_left_hr - P_right_hr);
      RHR = RHR - mi_hr;

      printSerial();
      firstSpecutrumDone = true;

      variablesToZero();            // reset variables
      LEDstatus(err_rr, err_hr);    // show results status
      beaconPowerLogic(alarmSignal, beaconStatus, beaconPowering);  // manage beacon power
    }
    if (iHrRr > ZP)
      iHrRr = 0;

    canSample = false;
    sei();
  } // if(canSample == true)
} // loop


/*### FUNCTIONS ###*/

void variablesToZero(void)
{
  for (int i = 0; i < FFT_N; i++) {
    data_rr[i] = 0;
    data_hr[i] = 0;
  }
  iHrRr = 0;
  sumS_rr = 0;
  sumS_hr = 0;
  sum3_rr = 0;
  sum3_hr = 0;
  maxi_rr = 0;
  maxi_hr = 0;
  maxiin_rr = 0;
  maxiin_hr = 0;
} // variablesToZero


int calculate_error_rr()
{
  int errorCode = 30;
  if (firstSpecutrumDone == true) // the first array is discarded because of noise
  {
    if (maxSP_rr > 40 and RRR < 60 and (float) sum3_rr / sumS_rr >= RATIO_RR) // signal is strong enough
    {
      if (RRR > 25)   // fast breathing
        errorCode = 2;
      else if (RRR >= 12 and RRR <= 25) // normal breathing
        errorCode = 0;
      else if (RRR > 3 and RRR < 12)   // slow breathing, >3 for filtering invalid results
        errorCode = 1;
      else
        errorCode = 0;  // invalid result
    }
    else
      errorCode = -1;   // low signal
  }
  return errorCode;
} // calculate_error_rr


int calculate_error_hr()
{
  int errorCode = 30;
  if (firstSpecutrumDone == true) // the first array is not considered because of noise
  {
    if (maxSP_hr > 40 and (float) sum3_hr / sumS_hr > RATIO_HR) // signal is strong enough
    {
      if (RHR > 100)  // high pulse
        errorCode = 2;
      else if (RHR >= 60 and RHR <= 100) // normal pulse
        errorCode = 0;
      else if (RHR > 20 and RHR < 60) // low pulse, >20 for filtering invalid results
        errorCode = 1;
      else
        errorCode = -1;  // invalid result
    }
    else
      errorCode = -1;   // signal is weak
  }
  return errorCode;
} // calculate_error_hr

float findRate(int8_t index)
{
  float rate = 60 * ((float(index) * float(fs)) / float(FFT_N));
  return rate;
} // findRate

const int intFaster = 250;
const int intSlower = 500;

void LEDstatus(int errorCode_rr, int errorCode_hr)
{
  bool ledState_rr, ledState_hr = LOW;
  intr_count = 0;
  LEDsOff();
  sei();

  while (intr_count < LEDinterval)
  {
    // RR
    if (errorCode_rr == 0)          // green on
      digitalWrite(LEDrr_gr, LOW);
    else if (errorCode_rr == 1 and intr_count % intSlower == 0) {   // red blinking slower
      digitalWrite(LEDrr_red, ledState_rr);
      ledState_rr = !ledState_rr;
    }
    else if (errorCode_rr == 2 and intr_count % intFaster == 0) {   // red blinking faster
      digitalWrite(LEDrr_red, ledState_rr);
      ledState_rr = !ledState_rr;
    }
    else if (errorCode_rr == -1) {  // red on
      digitalWrite(LEDrr_red, LOW);
      alarmSignal = true;
    }

    // HR
    if (errorCode_hr == 0)          // green on
      digitalWrite(LEDhr_gr, LOW);
    else if (errorCode_hr == 1 and intr_count % intSlower == 0) {   // red blinking slower
      digitalWrite(LEDhr_red, ledState_hr);
      ledState_hr = !ledState_hr;
    }
    else if (errorCode_hr == 2 and intr_count % intFaster == 0) {   // red blinking faster
      digitalWrite(LEDhr_red, ledState_hr);
      ledState_hr = !ledState_hr;
    }
    else if (errorCode_hr == -1) {  // red on
      digitalWrite(LEDhr_red, LOW);
      alarmSignal = true;
    }

    // Temperature
    if (Tcel <= 37.2)
      digitalWrite(LEDt_gr, LOW);   // green on
    else {
      digitalWrite(LEDt_red, LOW);  // red on
      alarmSignal = true;
    }
  }

  LEDsOff();
  intr_count = 0;
  cli();
} // LEDstatus


void LEDsOff()
{
  digitalWrite(LEDt_gr, HIGH);
  digitalWrite(LEDt_red, HIGH);
  digitalWrite(LEDrr_gr, HIGH);
  digitalWrite(LEDrr_red, HIGH);
  digitalWrite(LEDhr_gr, HIGH);
  digitalWrite(LEDhr_red, HIGH);
} // LEDsOff


void beaconPowerLogic(bool alarmSignal, bool beaconStatus, bool beaconPowering)
{
  if (alarmSignal == true and beaconStatus == false and beaconPowering == false) { // low signal, beacon is off and turning on is not initiated: it needs to turn on
    digitalWrite(BEACONpin, HIGH);      // high state turns on the switch
    beaconPowering = true;              // beacon is in process of turning on
    beaconPower_count = sample_count;   // 3 seconds is needed to turn it on or off
  }
  if (beaconPowering == true and beaconStatus == false and sample_count - beaconPower_count >= 32 * 3) { // 3 seconds have passed, beacon is now on
    digitalWrite(BEACONpin, LOW);       // turn off the switch
    beaconPowering = false;             // beacon is not in process of turning on
    beaconStatus = true;                // beacon is on
  }
  if (alarmSignal == false and beaconStatus == true and beaconPowering == false) { // alarmSignal changed to false, beacon needs to turn off
    digitalWrite(BEACONpin, HIGH);      // the same switch is used for turning off
    beaconPowering = true;              // beacon is in process of turning off
    beaconPower_count = sample_count;   // for measuring another 3 seconds to make sure it is off
  }
  if (beaconPowering == true and beaconStatus == true and sample_count - beaconPower_count >= 32 * 3) { // 3 seconds have passed, beacon is now off
    digitalWrite(BEACONpin, LOW);       // turn off the switch
    beaconPowering = false;             // beacon is not in process of turning on or off
    beaconStatus = false;               // beacon is off
  }
} // beaconPowerLogic


// Timer 1 function
void comp_match()
{
  TCCR1A = 0;
  TCCR1B = 1 << WGM12 | 0 << CS12 | 1 << CS11 | 1 << CS10; // CTC mode, prescaler = 64
  TCNT1 = 0;              // Reset Timer 1 counter
  // OCR1A = ((F_clock / prescaler) / Fs) - 1 = 2499
  OCR1A = 249;            // Set sampling frequency Fs = 1000 Hz
  TIMSK1 = 1 << OCIE1A;   // Enable Timer 1 interrupt
} // comp_match

// Timer 1 interrupt function
ISR (TIMER1_COMPA_vect)   // Interapt tajmera, podesen na 1ms
{
  if (intr_count == Tms && canSample == false) { // 1000ms = 1Hz
    intr_count = 0;       // reset intr_count to continue the process
    canSample = true;
  }
  else
    intr_count++;         // increment intr_count each 1ms

} // ISR


void ReadInternal(int* RR, int* HR)
{
  *RR = analogRead(RRpin);
  *HR = analogRead(HRpin);
}

//coeficient beta in HP filter
float calculate_beta(float fc, int fs) {
  float beta;
  beta = 1 / ((2 * PI * fc / (float)fs) + 1);
  return beta;
}

// HP filter of 1st order
float high_pass_rr(float beta, float x)
{
  float y = 0;
  int y_out;
  y = beta * y_hp_rr + beta * (x - x_hp_rr);
  y_hp_rr = y;
  x_hp_rr = x;

  y_out = (int)y;
  return y_out;
} // high_pass

float high_pass_hr(float beta, float x)
{
  float y = 0;
  int y_out;
  y = beta * y_hp_hr + beta * (x - x_hp_hr);
  y_hp_hr = y;
  x_hp_hr = x;

  y_out = (int)y;
  return y_out;
} // high_pass


int limit(int x, int a, int b)
{
  if (x < a)
    return a;
  else if (x > b)
    return b;
  else
    return x;
}

void printSerial()
{
  Tcel = sensors.getTempCByIndex(0);
  // calculate error codes
  err_rr = calculate_error_rr();
  err_hr = calculate_error_hr();

  if (err_rr != -1) {
    Serial.print(RRR);
    Serial.print(", ");
  }
  else {
    Serial.print(-1);
    Serial.print(", ");
  }
  
  if (err_hr != -1) {
    Serial.print(RHR);
    Serial.print(", ");
  }
  else {
    Serial.print(-1);
    Serial.print(", ");
  }

  Serial.print(Tcel);
  Serial.print("\r\n");
} // printSerial

