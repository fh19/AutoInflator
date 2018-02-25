/* Automatic tire inflation */

#define TIRE_INFLATER_Date    "2018-02-25"
#define TIRE_INFLATER_Author  "Falk Hecker"
#define TIRE_INFLATER_Version "V0.61"

/*
  Functions:
  -----------
  - Automatic inflation of tires
  - read pressure in filling pipe
  - if pressure is higher than threshold
   -> inflate/deflate until desired pressure is achieved
  - 2 Valves:
    o #1 for inflation
    o #2 for deflation
  - LCD 5110
  - on-switch is a double switch:
    o 1st to switch on power (parallel to ON_RELAY)
    o 2nd connected to sw_on (to change mode from automatic to manual operation and back)
  - plus+minus switch to set pressure or for manual operation
  - all switches connect to ground (push button)



  Changelog:
  ----------

  V0.1 (2018-01-28):
  - Initialversion

  V0.2. 0.3 (2018-02-11):
  - switches per interrupt

  V0.4 (2018-02-18):
  - display optimized
  - control loop changed

  V0.5 (2018-02-18):
  - first working sample
  - manual control via interrupt
  - contropl loop improved (ratio correction)
  - manual switch off

  V0.6 (2018-02-25):
  - contropl loop improved (ratio optimization)
  - open/close-time of valve changed from 10->20ms
*/

//#define DEBUG_ON


// Multiplikatoren definieren
#define SEKUNDEN (1000L)
#define MINUTEN  (60L * SEKUNDEN)
#define STUNDEN  (60L * MINUTEN)
#define TAGE     (24L * STUNDEN)
#define WOCHEN   (7L * TAGE)
#define MONATE   (30L * TAGE)
#define JAHRE    (365L * TAGE)


/*------------------------------------------------------------------*
  Timing fuer Befuellen
  ------------------------------------------------------------------*/


/*------------------------------------------------------------------*
  Ende Timing
  ------------------------------------------------------------------*/

#include <LCD5110_Basic.h>
#include <EEPROM.h>

extern uint8_t SmallFont[];
extern uint8_t TinyFont[];
extern uint8_t MediumNumbers[];
extern uint8_t BigNumbers[];


// control of on/off
#define T_AUTO_OFF    (15L * MINUTEN) // Zeit bis Gerät automatisch ausschaltet (nach Untaetigkeit)


// definition of control behavior
#define T_TEST_PULSE    (10)   // length of first test pulse
#define T_OPEN_CLOSE    (20)   // Time to open and close the valve (w/o effect on air flow)
#define T_CALM_TIME     (1000) // time to calm down pressure

#define NP_MEAS         10     // No of measurements to average pressure values

// behavior in automatic mode
#define MAX_T_AUTOMATIC (30L * SEKUNDEN)
#define DP_TOL          0.02   // Tolerance to finish automatic inflation [bar]
#define REL_DP_TOL      0.005  // rel. tolerance: dp_tol = DP_TOL + REL_DP_TOL * p_set
#define P_RATIO_ADAPT   0.1    // factor defining which amount of pressure ratio correction should be used: 0.2 means 20%

// behavior w/ plus/minus
#define P_INCR        0.1   // increment for set pressure change
#define P_MIN         0.3   // min. pressure 
#define P_MAX         8.5   // max. pressure



// decoupling time for switches
#define T_PRELL    250  // Entprellzeit [ms]


/*------------------------------------------------------------------*
  Port assignment
  ------------------------------------------------------------------*/

// Ports for LCD 5110
#define LCD_CLK       SCK
#define LCD_DIN       MOSI
#define LCD_DC        A2
#define LCD_RST       10
#define LCD_CE        SS
#define LCD_LED       A3

// analogue input for pressure measurement
#define E_P0          A8
#define E_POUT       A11

// ports for valves
#define VALVE1        6  // Valve for inflation
#define VALVE2        7  // Valve for deflation

// switch inputs (switch pull port to gnd)
#define SW_PLUS        3
#define SW_MINUS       2
#define SW_ON          1

// Output for Tone
#define LS             9

// Relais zum Ein- und Ausschalten
#define ON_RELAY     A5  // Relais zum Stromeinschalten

/*------------------------------------------------------------------*
  END Port assignement
  ------------------------------------------------------------------*/


// further definitions
// Power-switch
#define P_ON        HIGH
#define P_OFF        LOW

// Valves
#define V1_OPEN      HIGH
#define V1_CLOSE     LOW
#define V2_OPEN      HIGH
#define V2_CLOSE     LOW


// calculation of analogue input voltages
// ADC: 5V / 1023Bit
#define U_Ain(i) (5.0 * float(i) / 1023.0)

// calculation of pressure

// pressure sensor characteristics
#define U0_0bar      0.679 //0.696   // Spannung bei 0 bar
#define U0_6bar      2.553 //2.570   // Spannung bei 6 bar
#define P0_FACT      (6.0 / (U0_6bar - U0_0bar))
#define P0_U(U) (((U) - U0_0bar) * P0_FACT)
#define P0_Ain(i) (P0_U(U_Ain(i)))

#define UOUT_0bar    0.694 //0.698   // Spannung bei 0 bar
#define UOUT_6bar    2.566 //2.570   // Spannung bei 6 bar
#define POUT_FACT    (6.0 / (UOUT_6bar - UOUT_0bar))
#define POUT_U(U) (((U) - UOUT_0bar) * POUT_FACT)
#define POUT_Ain(i) (POUT_U(U_Ain(i)))

// switch states
#define NOT_PRESSED    0
#define PRESSED        1


// define States for main state machine
#define WAITING     0
#define INFLATING   1
#define DEFLATING   2
#define MANUAL      3
#define AUTOMATIC   4
#define SLEEPING    5
#define SW_OFF     10



/*
  Initialize LCD
  CLK, DIN, DC, RST, CE
  LCD5110 LC_Display(1,   0,  A3,  A5, A4);
*/
LCD5110 LC_Display(LCD_CLK, LCD_DIN, LCD_DC, LCD_RST, LCD_CE);



/*------------------------------------------------------------------*
  EEPROM-handling (for set value and some statistics)
  ------------------------------------------------------------------*/
#define WIDTH  (8 * sizeof(uint8_t))
#define TOPBIT (1 << (WIDTH - 1))
#define POLYNOMIAL 0xd5 /* CRC-8 Polynom */

// Bytes for EEPROM storage
#define NEEPROM 100

struct data_t {
  unsigned long N_Inflations;    // number of inflation processes
  float p_set;               // last set pressure
  unsigned long t_operation; // operating time [s]
  unsigned N_operation;      // number of power on cycles
};

union {
  byte b[NEEPROM];
  data_t data;
}
statistics;

unsigned int N_Inflations = 0;


// EEPROM-functions
void init_statistics();
void read_EEPROM();
void write_EEPROM();
uint8_t get_crc(uint8_t const message[], int nBytes);

/*------------------------------------------------------------------*
  END EEPROM-handling
  ------------------------------------------------------------------*/


/*------------------------------------------------------------------*
  function declaration
  ------------------------------------------------------------------*/

void beep_ready();
void beep_stopped();

// valve handling
void valve_open(int v, unsigned long t_open = 10);
void valve_close(int v);

// enable/disable interrups for plus+minus switch
void attach_int();
void detach_int();

// ISR for switch handling
void sw_plus();
void sw_minus();
void sw_on();

// read switch status
int status_plus();
int status_minus();
int status_on();

// show pressures
void print_p();

// clear pressure display
void clr_p();

// wait for some time and read+display pressures
void delay_valve(unsigned long t_open);

// read pressures np_meas times
void get_p(unsigned int np_meas);


/*------------------------------------------------------------------*
  END function declaration
  ------------------------------------------------------------------*/



/*------------------------------------------------------------------*
  variable declaration
  ------------------------------------------------------------------*/

volatile int
state = 0;

unsigned long
t_open,
t_AUTOMATIC = 0,
t_last_print = 0,
t_operation = 0; // Betriebszeit [s]


volatile unsigned long
t_auto_off = 0,
t_plus = 0,
t_minus = 0,
t_on = 0;

volatile float
p_set = 2.0;


float
dp_tol = DP_TOL,
p_0 = 0.0,
p_old = 0.0,
p_out = 0,
dp = 0.0,
dp_old = 0.0;


/*------------------------------------------------------------------*
  END variable declaration
  ------------------------------------------------------------------*/



/*------------------------------------------------------------------*
   function implementation
  ------------------------------------------------------------------*/

// init-routine
void setup()
{

  t_operation = millis(); // operating time [ms]

  // port to switch entire device on
  pinMode(ON_RELAY, OUTPUT);
  digitalWrite(ON_RELAY, P_ON);   // Relais ein

  pinMode(LCD_LED, OUTPUT);
  digitalWrite(LCD_LED, HIGH);   // LCD-Beleuchtung an
  LC_Display.InitLCD();
  LC_Display.clrScr();
  LC_Display.setFont(SmallFont);
  LC_Display.setContrast(40);
  LC_Display.print("Tire Inflater", CENTER, 0);
  LC_Display.print(TIRE_INFLATER_Author, CENTER, 16);
  LC_Display.print(TIRE_INFLATER_Version, CENTER, 24);
  LC_Display.print(TIRE_INFLATER_Date, CENTER, 32);
  delay(1000);



#ifdef DEBUG_ON
  // initialize serial communications at 9600 bps
  Serial.begin(9600);
#endif

  LC_Display.clrScr();
  LC_Display.setFont(SmallFont);
  LC_Display.print("Initialize ports", LEFT, 0);

  // pressure inputs
  pinMode(E_P0, INPUT);
  pinMode(E_POUT, INPUT);

  // valves
  pinMode(VALVE1, OUTPUT);
  digitalWrite(VALVE1, V1_CLOSE);
  pinMode(VALVE2, OUTPUT);
  digitalWrite(VALVE2, V2_CLOSE);

  // switches (input pullup)
  pinMode(SW_MINUS, INPUT_PULLUP);
  digitalWrite(SW_MINUS, HIGH);
  pinMode(SW_PLUS, INPUT_PULLUP);
  digitalWrite(SW_PLUS, HIGH);
  pinMode(SW_ON, INPUT_PULLUP);
  digitalWrite(SW_ON, HIGH);

  // speaker
  pinMode(LS, OUTPUT);
  digitalWrite(LS, LOW);

  LC_Display.print(" done", RIGHT, 0);


  LC_Display.clrScr();

  // read EEPROM
  read_EEPROM();

  // increase pw-on counter
  statistics.data.N_operation += 1;

  // show EEPROM-data...
  LC_Display.clrScr();
  LC_Display.setFont(SmallFont);
  LC_Display.print("Set:", LEFT, 8);
  LC_Display.print("bar", RIGHT, 8);
  LC_Display.printNumF(statistics.data.p_set, 2, 30, 8, '.', 5, ' ');
  LC_Display.print("Inflations:", LEFT, 16);
  LC_Display.printNumI(statistics.data.N_Inflations, LEFT, 24, 7, '0');
  LC_Display.print("Operations:", LEFT, 32);
  LC_Display.printNumI(statistics.data.N_operation, LEFT, 40, 7, '0');

  t_auto_off = millis();

  delay(1000);

  // init display
  LC_Display.clrScr();
  LC_Display.setFont(SmallFont);
  LC_Display.print("P0:", LEFT, 0);
  LC_Display.print("bar", RIGHT, 0);
  LC_Display.print("Set:", LEFT, 8);
  LC_Display.print("bar", RIGHT, 8);
  LC_Display.print("bar", RIGHT, 40);
  get_p(5);

  // wait until ON-button is released
  while (status_on() == PRESSED) {  // manual inflation w/ button 3
    get_p(50);
  }

  // wait another second until device is ready
  delay(1000);
  // start interrupt-handling for switches
  attachInterrupt(digitalPinToInterrupt(SW_ON),    sw_on,    FALLING);
  state = WAITING;
  attach_int();

}



// enable interrupts for +/- switches
void attach_int()
{
  attachInterrupt(digitalPinToInterrupt(SW_PLUS),  sw_plus,  FALLING);
  attachInterrupt(digitalPinToInterrupt(SW_MINUS), sw_minus, FALLING);
}


// disable interrupts for +/- switches
void detach_int()
{
  detachInterrupt(digitalPinToInterrupt(SW_PLUS));
  detachInterrupt(digitalPinToInterrupt(SW_MINUS));
}

// ISR for plus-switch
void sw_plus()
{
  if (millis() > t_plus + T_PRELL)
  {
    p_set = p_set + P_INCR;
    if (p_set > P_MAX) p_set = P_MIN;
    tone(LS, 1200, 70);
    //  beep();
    t_plus = millis();
    // reactivate control loop again when set value was changed
    if (state == SLEEPING) {
      state = WAITING;
    }
    t_auto_off = millis();

  }
}


// ISR for minus-switch
void sw_minus()
{
  if (millis() > t_minus + T_PRELL)
  {
    p_set = p_set - P_INCR;
    if (p_set < P_MIN) p_set = P_MAX;
    tone(LS, 800, 70);
    //  beep();
    t_minus = millis();
    // reactivate control loop again when set value was changed
    if (state == SLEEPING) {
      state = WAITING;
    }
    t_auto_off = t_minus;
  }
}



// ISR for on-switch (for use as mode-switch)
void sw_on()
{
  if (millis() > t_on + 2 * T_PRELL)
  {
    // double push of on-sw = switch off
    if (millis() < t_on + 1000) {
      state = SW_OFF;
    } else {
      tone(LS, 400, 100);
      t_on = millis();
      if (state != MANUAL) {
        detach_int();
        state = MANUAL;
      } else if (state == MANUAL) {
        attach_int();
        state = WAITING;
      }
      t_auto_off = t_on;
    }
  }
}




// poll switch status
int status_plus()
{
  int i = 1 - digitalRead(SW_PLUS);

#ifdef DEBUG_ON
  Serial.print("Plus: ");
  Serial.println(i);
#endif

  return (i);
}

// poll switch status
int status_minus()
{
  int i = 1 - digitalRead(SW_MINUS);

#ifdef DEBUG_ON
  Serial.print("Minus: ");
  Serial.println(i);
#endif

  return (i);
}


// poll switch status
int status_on()
{
  int i = 1 - digitalRead(SW_ON);

#ifdef DEBUG_ON
  Serial.print("SW_ON: ");
  Serial.println(i);
#endif

  return (i);
}


/*
   The width of the CRC calculation and result.
   Modify the typedef for a 16 or 32-bit CRC standard.
*/
uint8_t get_crc(uint8_t const message[], int nBytes)
{
  uint8_t remainder = 0;

  // Perform modulo-2 division, a byte at a time.
  for (int byte = 0; byte < nBytes; ++byte) {
    // Bring the next byte into the remainder.
    remainder ^= (message[byte] << (WIDTH - 8));

    // Perform modulo-2 division, a bit at a time.
    for (uint8_t bit = 8; bit > 0; --bit) {
      // Try to divide the current data bit.
      if (remainder & TOPBIT) {
        remainder = (remainder << 1) ^ POLYNOMIAL;
      }
      else {
        remainder = (remainder << 1);
      }
    }
  }

  // The final remainder is the CRC result.
  return (remainder);

}   /* crcSlow() */


// initialize EEPROM data
void init_statistics()
{
  for (int i = 0; i < NEEPROM; i++) statistics.b[i] = 0;
  statistics.data.N_Inflations = 0;                // Anzahl Gatterstarts
  statistics.data.p_set = p_set;                  // last set pressure
  statistics.data.t_operation = millis() / 1000; // Betriebszeit [s]
  statistics.data.N_operation = 0;             // Anzahl Einschaltzyklen
}


// read and verify EEPROM
// data is stored twice, if one segment is faulty => other is taken
void read_EEPROM()
{
  LC_Display.setFont(SmallFont);
  LC_Display.print("Read EEPROM...", LEFT, 0);
  for (int i = 0; i < NEEPROM; i++) {
    statistics.b[i] = EEPROM.read(i);
  }
  uint8_t crc0 = EEPROM.read(NEEPROM);
  uint8_t crc1 = get_crc(statistics.b, NEEPROM);

  LC_Display.printNumI(crc0, LEFT, 16, 3, '0');
  LC_Display.printNumI(crc1, RIGHT, 16, 3, '0');

  // wenn Checksumme falsch => Kopie lesen
  if (crc0 != crc1) {
    LC_Display.print("Err", 50, 0);
    LC_Display.print("Read copy", LEFT, 8);
    for (int i = 0; i < NEEPROM; i++) {
      statistics.b[i] = EEPROM.read(i + 1 + NEEPROM);
    }
    crc0 = EEPROM.read(2 * NEEPROM + 1);
    crc1 = get_crc(statistics.b, NEEPROM);
    LC_Display.printNumI(crc0, LEFT, 24, 3, '0');
    LC_Display.printNumI(crc1, LEFT, 32, 3, '0');
    if (crc0 != crc1) {
      LC_Display.print("Err", 50, 8);
      init_statistics();
    } else {
      LC_Display.print("OK", 50, 8);
      p_set = min(statistics.data.p_set, 4.1);         // last set-pressure
    }
  } else {
    LC_Display.print("OK", 50, 0);
    p_set = min(statistics.data.p_set, 4.1);         // last set-pressure
  }
  delay(500);
}



/*
   write EEPROM twice including CRC
*/
void write_EEPROM()
{
  LC_Display.setFont(SmallFont);
  LC_Display.print("Write EEPROM...", LEFT, 0);

  statistics.data.t_operation = (millis() - t_operation) / 1000; // Betriebszeit [s]
  statistics.data.p_set = min(p_set, 8.0);         // last set-pressure
  uint8_t crc = get_crc(statistics.b, NEEPROM);
  for (int i = 0; i < NEEPROM; i++) {
    EEPROM.write(i, statistics.b[i]);
  }
  EEPROM.write(NEEPROM, crc);
  for (int i = 0; i < NEEPROM; i++) {
    EEPROM.write(i + 1 + NEEPROM, statistics.b[i]);
  }
  EEPROM.write(2 * NEEPROM + 1, crc);
  LC_Display.print("-OK", RIGHT, 0);

}



// show pressures
void print_p()
{
  if (millis() > (t_last_print + 100)) {
    clr_p();
    //    LC_Display.setFont(MediumNumbers);
    LC_Display.setFont(BigNumbers);
    LC_Display.printNumF(p_out, 2, LEFT, 16, '.', 5, '/');
    LC_Display.setFont(SmallFont);
    LC_Display.printNumF(p_0, 2, 30, 0, '.', 5, ' ');
    LC_Display.printNumF(p_set, 2, 30, 8, '.', 5, ' ');
    t_last_print = millis();
  }
}

// clear pressure display
void clr_p()
{
  LC_Display.setFont(BigNumbers);
  //  LC_Display.setFont(MediumNumbers);
  LC_Display.print("/////", LEFT, 40);
  LC_Display.setFont(SmallFont);
  LC_Display.print("      ", 30, 0);
  LC_Display.print("      ", 30, 8);
}



void beep_ready()
{
  tone(LS, 500, 500);
}

void beep_stopped()
{
  tone(LS, 500, 200);
  delay(200);
  tone(LS, 500, 200);
}


void delay_valve(unsigned long t_open)
{
  unsigned long t_end;
  t_end = millis() + t_open;
  while (millis() < t_end) {
    get_p(2);
  }
}



void valve_open(int v, unsigned long t_open)
{
  switch (v) {
    case VALVE1:  // Ventil fuer untere Kammer
      digitalWrite(VALVE1, V1_OPEN);     // Ventil an = Zylinder entlueften
      digitalWrite(VALVE2, V2_CLOSE);    // Sicherstellen, dass anderes Ventil geschlossen ist
      delay_valve(t_open + T_OPEN_CLOSE);
      valve_close(VALVE1);
      break;
    case VALVE2:  // Ventil fuer obere Kammer
      digitalWrite(VALVE1, V1_CLOSE);    // Sicherstellen, dass anderes Ventil geschlossen ist
      digitalWrite(VALVE2, V2_OPEN);     // Ventil an = Zylinder entlueften
      delay_valve(t_open + T_OPEN_CLOSE);
      valve_close(VALVE2);
      break;
  }
}


void valve_close(int v)
{
  switch (v) {
    case VALVE1:  // Ventil fuer untere Kammer
      digitalWrite(VALVE1, V1_CLOSE);     // Ventil an = Zylinder entlueften
      break;
    case VALVE2:  // Ventil fuer obere Kammer
      digitalWrite(VALVE2, V2_CLOSE);     // Ventil an = Zylinder entlueften
      break;
  }
}





// read pressures np_meas times
void get_p(unsigned int np_meas)
{
  unsigned long
  U0 = 0, U1 = 0;
  float U = 0;
  if (np_meas < 1) np_meas = 1;
  for (int i = 0; i < np_meas; i++) {
    U0 += analogRead(E_P0);
    delay(5);
    U1 += analogRead(E_POUT);
    delay(5);
  }
  U = 1.0 / np_meas * U0;
  p_0 = P0_Ain(U);
  U = 1.0 / np_meas * U1;
  p_out = POUT_Ain(U);
#ifdef DEBUG_ON
  Serial.print("p0(");
  Serial.print(U0 / np_meas);
  Serial.print("): ");
  Serial.print(int(1000 * p_0));
  Serial.print(" mbar, po(");
  Serial.print(U1 / np_meas);
  Serial.print("): ");
  Serial.print(int(1000 * p_out));
  Serial.println(" mbar");
#endif
  print_p();
}



void loop() {

  //  Serial.println("Startschleife");
  get_p(NP_MEAS);

  // check if supply pressure is sufficient
  if (    (state != MANUAL)
       && (state != SW_OFF)
       && (p_0 < 2.5) )               // supply pressure too low (lower than desired uotput)
  {
    state = SLEEPING;
    LC_Display.setFont(SmallFont);
    LC_Display.print("Low press.", LEFT, 40);
  }

  // Timeout => switch off
  if (millis() > ( T_AUTO_OFF + t_auto_off)) state = SW_OFF;


  // main state machine
  switch (state) {


    // reactivate only if pressure at output drops down to 0
    case SLEEPING:
      if (   (p_out < 0.1)
             && (p_0 > max((p_set + 0.5), 2.5)) )  // supply pressure sufficient
      {
        state = WAITING;
      }
      break;

    case WAITING:
      LC_Display.setFont(SmallFont);
      LC_Display.print("Waiting...", LEFT, 40);
      if ( (state == WAITING) && (p_out > (0.9 * P_MIN)) ) {
        state = AUTOMATIC;
        t_AUTOMATIC = millis();
      }
      break;


    case AUTOMATIC:
#ifdef DEBUG_ON
      Serial.println("Automatic...");
#endif
      t_auto_off = millis();
      dp = p_set - p_out;  // difference to set-pressure
      dp_tol = DP_TOL + p_set / 200.0;
      t_open = T_TEST_PULSE;
      LC_Display.setFont(SmallFont);
      LC_Display.print("Auto...   ", LEFT, 40);
      while (state == AUTOMATIC) {
        dp_tol = DP_TOL + REL_DP_TOL * p_set;
        p_old = p_out;
        //if (dp > dp_tol) {
        if (dp > DP_TOL) {  // low tolerance for inflation
          valve_open(VALVE1, t_open);
        } else if (dp < -dp_tol) {  // higher tolerance for deflation
          valve_open(VALVE2, t_open);
        } else {
          beep_ready();
          statistics.data.N_Inflations++;
          state = SLEEPING;
          LC_Display.setFont(SmallFont);
          LC_Display.print("Finished. ", LEFT, 40);
        }
        delay_valve(T_CALM_TIME);
        get_p(NP_MEAS);

        if (   //( (dp > dp_tol) && ( ((p_set - p_out)) < 0) ) // if pressure was supposed to go up but in fact decreased => target will never be reached
          (millis() > t_AUTOMATIC + MAX_T_AUTOMATIC)           // if we tried already too long
          || (p_out < 0.1)                                     // no pressure at output
          || (p_0 < p_set) || (p_0 < 2.5) )                    // supply pressure too low (lower than desired uotput) or lower than required for valves
        {
          state = SLEEPING;
          LC_Display.setFont(SmallFont);
          LC_Display.print("Stopped.  ", LEFT, 40);
          beep_stopped();
        }

        dp = p_set - p_out;  // difference to set-pressure
        dp_old = max(abs(p_old - p_out), 0.1);  // what was achieved by last pulse
        if (dp > 0) {        // inflation
          t_open = t_open * abs(dp) / abs(dp_old) 
                   * ( 1.0 - P_RATIO_ADAPT + P_RATIO_ADAPT * ( ((p_out + p_set) / 2.0 + 1) / ( (p_old + p_out) / 2.0 + 1) ));  // correction by change of pressure ratio
        } else {             // deflation
          t_open = t_open * abs(dp) / abs(dp_old) 
                   * ( 1.0 - P_RATIO_ADAPT + P_RATIO_ADAPT * ( ((p_old + p_out) / 2.0 + 1) / ( (p_out + p_set) / 2.0 + 1) ));  // correction by change of pressure ratio
        }
      }
      break;

    case MANUAL:

      LC_Display.setFont(SmallFont);
      LC_Display.print("Manual... ", LEFT, 40);

      while (state == MANUAL) {  // manual operation w/ button 3
        t_auto_off = millis();

        while (status_plus() == PRESSED) {  // manual inflation
          digitalWrite(VALVE2, V2_CLOSE);    // close valve
          digitalWrite(VALVE1, V1_OPEN);     // open valve
          delay_valve(10);
        }

        while (status_minus() == PRESSED) {  // manual deflation
          digitalWrite(VALVE1, V1_CLOSE);    // close valve
          digitalWrite(VALVE2, V2_OPEN);     // open valve
          delay_valve(10);
        }

        // close valves
        digitalWrite(VALVE1, V1_CLOSE);
        digitalWrite(VALVE2, V2_CLOSE);
        delay_valve(10);
      }
      LC_Display.setFont(SmallFont);
      LC_Display.print("Waiting...", LEFT, 40);
      delay_valve(T_CALM_TIME);

      break;


    // Geraet abschalten
    case SW_OFF:
      // kurzer Ton zur Bestaetigung
      tone(LS, 700, 250);
      valve_close(VALVE1);
      valve_close(VALVE2);

      delay(500);
      LC_Display.clrScr();
      write_EEPROM();
      tone(LS, 400, 500);
      delay(2000);
      tone(LS, 200, 1000);
      LC_Display.print("Switch Off...", 0, 8);
      delay(2000);
      LC_Display.print("** ------ **", 0, 16);
      digitalWrite(ON_RELAY, P_OFF);   // Relais Spannungsversorgung aus
      delay(2000);
      LC_Display.print("Reset...", 0, 24);
      asm volatile ("  jmp 0"); // reset();
      break;

    default:
      break;
  }

}

