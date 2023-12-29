// Syntrofos 1.1 time domain processing

#include <OneWire.h>
#include <DallasTemperature.h>


#define LEDinterval 300  // LED signal duration in ms
#define ONE_WIRE_BUS 2   // temperature sensor pin
#define HRpin A1         // HR sensor pin
#define RRpin A2         // RR sensor pin
#define LEDhr_gr 12      // HR green LED pin
#define LEDhr_r 11       // HR red LED pin
#define LEDrr_gr 10      // RR green LED pin
#define LEDrr_r 9        // RR red LED pin
#define LEDt_gr 8        // temperature green LED pin
#define LEDt_r 7         // temperature red LED pin
#define BEACONpin 6      // beacon pin


const byte fs = 32;       // sampling frequency
int Ts;                   // sampling period in ms
const int NUM_PULSES = 4; // number of pulses after which rhythm is calculated
int intr_count = 0;       // timer interrupts counter
bool canSample = false;   // sampling indicator

// filter variables
float alfa_hr, beta_hr;
float alfa_rr, beta_rr;
float y_lp1_hr, y_hp1_hr, x_hp1_hr = 0.0;
float y_lp1_rr, y_hp1_rr, x_hp1_rr = 0.0;

float Tcel = 0.0;               // temperature variable
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


void ckeckPulseBinHR(void);
void ckeckPulseBinRR(void);


void setup()
{
  const float fc_lp_hr = 1.0;
  const float fc_hp_hr = 0.5;
  const float fc_lp_rr = 1.0;
  const float fc_hp_rr = 0.4;

  pinMode(LEDhr_gr, OUTPUT);
  pinMode(LEDhr_r, OUTPUT);
  pinMode(LEDrr_gr, OUTPUT);
  pinMode(LEDrr_r, OUTPUT);
  pinMode(LEDt_gr, OUTPUT);
  pinMode(LEDt_r, OUTPUT);
  pinMode(BEACONpin, OUTPUT);
  LEDsOff();                   // set LEDs initially to off

  analogReference(INTERNAL);   // a built-in reference, equal to 1.1 volts on the ATmega168 or ATmega328P
  Serial.begin(115200);
  Serial.println("Syntrofos v1.1");

  cli();
  alfa_hr = calculate_alfa(fc_lp_hr, fs);
  beta_hr = calculate_beta(fc_hp_hr, fs);
  alfa_rr = calculate_alfa(fc_lp_rr, fs);
  beta_rr = calculate_beta(fc_hp_rr, fs);
  Ts = (int) (1000 / fs);
  delay(500);
  sensors.begin();
  sensors.setWaitForConversion(false); // if set to false then it is not blocking, but then 750ms or more must be waited between calls
  comp_match();
  sei();
} // setup


int HR, RR = 0;                             // heart rate and respiratory rate
int HRpulses, RRpulses = 0;                 // temporary pulse counters
int sample_count, status_count = 0;         // sample counter
uint8_t pulseBin_hr, pulseBin_old_hr = 0;   // pulse heart signal new and old values
uint8_t pulseBin_rr, pulseBin_old_rr = 0;   // pulse respiratory signal new and old values
int HRpulse_T[2], RRpulse_T[2] = {0, 0};    // for storing treshold-passing time to calculate period
int hr_ti, rr_ti = 0;                       // HRpulse_T and RRpulse_T iterators
int HRpulse_T_start, RRpulse_T_start;       // for saving time when first pulse is detected
int hr_count, rr_count = 0;                 // for saving time when preset number of pulses was detected
bool ledstatus = false;                     // false: LEDs blink when signal changes, true: showign status, constant green or red on
bool alarmSignal = false;                   // if any of 3 monitored signals is out of safe range
bool beaconStatus, beaconPowering = false;  // false: off, true: on
int beaconPower_count = 0;                  // to count beacon powering duration
bool isGoodHsignal, isGoodRsignal;

void loop()
{
  int sampleRR, sampleHR;
  float y_lp_hr, y_hp_hr;
  float y_lp_rr, y_hp_rr;
  int th_hr, th_rr;

  if (canSample == true)
  {
    cli();
    ReadInternal(&sampleHR, &sampleRR); // take samples
    sample_count++;                     // count samples

    // filter samples and return pulse signal
    pulseBin_hr = filter_hr(sampleHR, &y_lp_hr, &y_hp_hr, &th_hr);
    pulseBin_rr = filter_rr(sampleRR, &y_lp_rr, &y_hp_rr, &th_rr);
    // detect rising edge, check periods and count pulses
    ckeckPulseBinHR();
    ckeckPulseBinRR();

    if (HRpulses == NUM_PULSES + 1) {                             // desired number of regular pulses is found
      HR = findRate(&HRpulses, &HRpulse_T_start, &sample_count);  // calculate the rhythm
      hr_count = sample_count;                                    // time when the 5th pulse is found
    }
    if (RRpulses == NUM_PULSES + 1) {
      RR = findRate(&RRpulses, &RRpulse_T_start, &sample_count);
      rr_count = sample_count;
    }
    if (sample_count - hr_count > fs * 25) {  // no heart signal for 25 seconds
      HR = 0;
      alarmSignal = true;
    }
    if (sample_count - rr_count > fs * 25) {  // no respiratory signal for 25 seconds
      RR = 0;
      alarmSignal = true;
    }

    printToSerial(&y_hp_hr, &y_hp_rr);  // optionally, print results to Serial oscilloscope or monitor to view or log waveforms and results
    beaconPowerLogic(alarmSignal, beaconStatus, beaconPowering);  // manage beacon power
    if (ledstatus == false)
      LEDsOff();                        // temporarily switch LEDs off for power saving, this creates fast blinking effect

    // end 10 seconds cycle by showing LED status
    if (sample_count % (fs * 10) == (fs * 9))   // ask for temperature 1 second earlier
      sensors.requestTemperatures();
    else if (sample_count % (fs * 10) == 0) {   // update temperature each 10 seconds
      Tcel = sensors.getTempCByIndex(0);
      sei();                        // disable interrupts while LED status is on
      ledstatus = true;             // LEDs will now show results status color
      status_count = sample_count;  // to monitor status duration
      LEDstatus(HR, RR);            // turn on the LEDs to green or red according to results
    }
    if (ledstatus == true and (sample_count - status_count) >= (32 * 4)) {  // 4 seconds of showing status have ended
      ledstatus = false;                                                    // LEDs will now blink at the rhythms of breathing and heart beating
      LEDsOff();
    }

    canSample = false;  // waiting for the next interrupt
    sei();              // enabling interrupts
  } // if(canSample == true)
} // loop


/*### FUNCTIONS ###*/

void printToSerial(float* y_hp_hr, float* y_hp_rr)
{
  Serial.print(Tcel);
  Serial.print(", ");
  Serial.print(HR);
  Serial.print(", ");
  Serial.print(RR);
  Serial.print(", ");
  Serial.print(*y_hp_hr);
  Serial.print(", ");
  Serial.print(*y_hp_rr);;
  Serial.print("\r\n");
} // printToSerial


// Timer 1 function
void comp_match()
{
  // Configure Timer 1 interrupt
  // F_clock = 16 MHz, prescaler = 64, Fs = 1000 Hz
  TCCR1A = 0;
  TCCR1B = 1 << WGM12 | 0 << CS12 | 1 << CS11 | 1 << CS10;
  TCNT1 = 0;              // reset Timer 1 counter
  // OCR1A = ((F_clock / prescaler) / Fs) - 1 = 2499
  OCR1A = 249;           // set sampling frequency Fs = 1000 Hz
  TIMSK1 = 1 << OCIE1A;  // enable Timer 1 interrupt
} // comp_match


// timer interrupt function set to execute each 1ms
ISR (TIMER1_COMPA_vect)
{
  if (intr_count == Ts && canSample == false) {
    intr_count = 0;    // reset intr_count to continue the process
    canSample = true;
  }
  else intr_count++;   // increment intr_count each 1ms
} // ISR


void ReadInternal(int* HR, int* RR)
{
  *RR = analogRead(RRpin);
  *HR = analogRead(HRpin);
} // ReadInternal


//coeficiant alfa for LP filter
float calculate_alfa(float fc, float fs) {
  float alfa;
  alfa = (2 * PI * fc / fs) / ((2 * PI * fc / fs) + 1);
  return alfa;
} // calculate_alfa

//coeficient beta in HP filter
float calculate_beta(float fc, int fs) {
  float beta;
  beta = 1 / ((2 * PI * fc / (float)fs) + 1);
  return beta;
} // calculate_beta


int32_t ydc_old_hr, ydc_old_rr = 0;

int dcTracking(int x, int32_t* ydc_old)
{
  int32_t ydc;
  ydc = *ydc_old + ((((int32_t) x << 16) - (*ydc_old)) >> 9);
  *ydc_old = ydc;
  return (ydc >> 16);
} // DC_Tracking


uint8_t filter_hr(int sample, float* y_lp_hr, float* y_hp_hr, int* th_hr)
{
  *y_lp_hr = low_pass(&alfa_hr, (float)sample, &y_lp1_hr);
  float x = *y_lp_hr;
  *y_hp_hr = high_pass(&beta_hr, &x, &x_hp1_hr, &y_hp1_hr);
  *th_hr = dcTracking(*y_hp_hr, &ydc_old_hr);

  // set LED indicator and return binary value
  if (*y_hp_hr > *th_hr + 10) {
    if (ledstatus == false)         // LEDs are configured to blink
      digitalWrite(LEDhr_gr, LOW);  // turn on the HR LED
    return 100;                     // high state
  }
  else {
    if (ledstatus == false)
      digitalWrite(LEDhr_gr, HIGH); // turn off the HR LED
    return 0;                       // low state
  }
} // filter_hr

float low_pass(float* alfa, float x, float* y_old)
{
  float y = 0;
  y = (*alfa) * x + (1.0 - (*alfa)) * (*y_old);
  *y_old = y;
  return y;
} // low_pass

float high_pass(float* beta, float *x, float* x_old, float* y_old)
{
  float y = 0;
  y = (*beta) * (*y_old) + (*beta) * ((*x) - (*x_old));
  *y_old = y;
  *x_old = *x;
  return y;
} // high_pass


uint8_t filter_rr(int sample, float* y_lp_rr, float* y_hp_rr, int* th_rr)
{
  *y_lp_rr = low_pass(&alfa_rr, (float)sample, &y_lp1_rr);
  float x = *y_lp_rr;
  *y_hp_rr = high_pass(&beta_rr, &x, &x_hp1_rr, &y_hp1_rr);
  *th_rr = dcTracking(*y_hp_rr, &ydc_old_rr);

  // set LED indicator and return binary value
  if (*y_hp_rr > *th_rr + 25) {
    if (ledstatus == false)         // if LEDs are configured to blink
      digitalWrite(LEDrr_gr, LOW);  // turn on the RR LED
    return 100;
  }
  else {
    if (ledstatus == false)
      digitalWrite(LEDrr_gr, HIGH); // turn off the RR LED
    return 0;
  }
} // filter_rr


void ckeckPulseBinHR()
{
  if (isRisingEdge(&pulseBin_old_hr, &pulseBin_hr) == 1) {
    HRpulse_T[hr_ti++] = sample_count;  // save time when rising edge is found
    hr_ti %= 2;                         // limit index to 0 or 1
    HRpulses++;                         // count pulses to calculate heart rate
    
    if (HRpulses == 1)
      HRpulse_T_start = sample_count;   // save time when first pulse is found

    isGoodHsignal = checkT(HRpulse_T, 10, 64);  // check if period is within expected range
    if (isGoodHsignal == false)                 // irregular pulse is detected because of noise or sensor is detached
      HRpulses = 0;                             // reset temporary counter
  }
} // ckeckPulseBinHR

void ckeckPulseBinRR()
{
  if (isRisingEdge(&pulseBin_old_rr, &pulseBin_rr) == 1) {
    RRpulse_T[rr_ti++] = sample_count;   // save time when rising edge is found
    rr_ti %= 2;                          // limit index to 0 or 1
    RRpulses++;                          // count pulses to calculate respiratory rate
    
    if (RRpulses == 1)
      RRpulse_T_start = sample_count;    // save time when first pulse is found

    isGoodRsignal = checkT(RRpulse_T, 48, 128);  // check if period is within expected range
    if (isGoodRsignal == false)                  // irregular pulse detected because of noise or sensor is detached
      RRpulses = 0;                              // reset temporary counter
  }
} // ckeckPulseBinRR

int isRisingEdge(uint8_t* x_old, uint8_t* x_new)
{
  int y = 0;                        // y = 0 if not rising edge
  if ((*x_new) - (*x_old) == 100)   // rising edge - from 100 to 0
    y = 1;
  *x_old = *x_new;                  // update old value
  return y;
} // isRisingEdge

bool checkT(int* x, int a, int b)
{
  int t = abs(x[1] - x[0]);
  if (t < a or t > b)
    return false;
  else
    return true;
} // checkT

int findRate(int* x, int* Tstart, int* Tfinal) {
  float y = 1000.0 * (((float)NUM_PULSES * 60.0) / ((float)(*Tfinal - *Tstart) * (float)Ts));
  *x = 0;
  return (int)y;
} // findRate


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


void LEDsOff()
{
  digitalWrite(LEDt_gr, HIGH);
  digitalWrite(LEDt_r, HIGH);
  digitalWrite(LEDrr_gr, HIGH);
  digitalWrite(LEDrr_r, HIGH);
  digitalWrite(LEDhr_gr, HIGH);
  digitalWrite(LEDhr_r, HIGH);
} // LEDsOff


void LEDstatus(int HR, int RR)
{
  LEDsOff();            // turn off all LEDs, in case if any remained on
  alarmSignal = false;  // any signal can turn on the alarm

  // Heart rate
  if (HR >= 60 and HR <= 100)     // safe range
    digitalWrite(LEDhr_gr, LOW);  // green on
  else {
    digitalWrite(LEDhr_r, LOW);   // dangerously high or low pulse or sensor is detached, red on
    alarmSignal = true;           // alarm is on
  }

  // Respiratory rate
  if (RR >= 12 and RR <= 25)           // safe range
    digitalWrite(LEDrr_gr, LOW);       // green on
  // respiratory signal is slower than HR so it takes more time to count NUM_PULSES periods
  // if not given enough time, it can be misinterpreted as too low rate
  else if (sample_count >= fs * 25) {  // 25 seconds to make NUM_PULSES periods have passed
    digitalWrite(LEDrr_r, LOW);        // dangerously high or low pulse or sensor is detached, red on
    alarmSignal = true;                // alarm is on; if it was already on at HR point, nothing is changed
  }

  // Temperature
  if (Tcel > 37.2 or Tcel == DEVICE_DISCONNECTED_C) {  // temperature is higher or sensor is unplugged
    digitalWrite(LEDt_r, LOW);                         // red on
    alarmSignal = true;                                // alarm is on
  }
  else
    digitalWrite(LEDt_gr, LOW);  // temperature is good, green on
} // LEDstatus
