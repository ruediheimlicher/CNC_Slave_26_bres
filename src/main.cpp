#include <Arduino.h>
#include <avr/io.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rawhid.h"

#include "settings.h"

#include "lcd.h"

// USB
#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))
static volatile uint8_t buffer[32]={};
static volatile uint8_t sendbuffer[32]={};

// begin Ringbuffer
#define RINGBUFFERTIEFE 4
#define READYBIT   0       // buffer kann Daten aufnehmen
#define FULLBIT   1        // Buffer ist voll
#define STARTBIT   2       // Buffer ist geladen
#define RINGBUFFERBIT 3    // Ringbuffer wird verwendet
#define LASTBIT   4         // Letzter Abschnitt  ist geladen
#define ENDBIT   5          // Letzter Abschnitt  ist abgearbeitet
#define STOPBIT   6        // Ablauf stoppen
#define FIRSTBIT   7

uint8_t CNCDaten[RINGBUFFERTIEFE][33];
volatile uint16_t abschnittnummer=0;
volatile uint16_t endposition= 0xFFFF;
volatile uint8_t ladeposition=0;

volatile uint8_t ringbufferstatus=0x00;   

uint16_t AbschnittCounter=0;
volatile uint8_t liniencounter= 0;
// end Ringbuffer

// SPI
#define OSZIPORT           PORTD
#define OSZIPORTDDR        DDRD
#define OSZIPORTPIN        PIND
#define OSZI_PULS_A        0
#define OSZI_PULS_B        1

#define OSZIALO OSZIPORT &= ~(1<<OSZI_PULS_A)
#define OSZIAHI OSZIPORT |= (1<<OSZI_PULS_A)
#define OSZIATOG OSZIPORT ^= (1<<OSZI_PULS_A)

#define OSZIBLO OSZIPORT &= ~(1<<OSZI_PULS_B)
#define OSZIBHI OSZIPORT |= (1<<OSZI_PULS_B)
#define OSZIBTOG OSZIPORT ^= (1<<OSZI_PULS_B)
// SPI
#define TIMER0_STARTWERT	0x40

#define LOOPLEDDDR          DDRF    //DDRD
#define LOOPLEDPORT         PORTF   //PORTD
#define LOOPLED             4       //6 

#define TASTENDDR           DDRF
#define TASTENPORT          PORTF
#define TASTENPIN          PINF

#define TASTE0				0   // HALT-PIN Motor A
#define TASTE1				1


#define STARTDELAYBIT       0
#define HICOUNTBIT          1

#define WDTBIT              7

// CNC12
#define CMD_PORT            PORTD   //    PORTB
#define CMD_DDR             DDRD    //    DDRB
#define CMD_PIN             PIND    //    PINB

#define LOAD_NEXT          5  // Bit fuer Laden des naechsten Abschnitts in der loop
#define LOAD_LAST          6  // Bit fuer Laden des letzten Abschnitts in der loop

// auf Stepperport 1
#define END_A0_PIN          6       //  PIN fuer Endanschlag A0 
#define END_B0_PIN          7 		//           Endanschlag B0 


// Auf Stepperport 2
#define END_C0_PIN          6       //  PIN fuer Endanschlag C0 
#define END_D0_PIN          7 		//           Endanschlag D0 


#define RICHTUNG_A	0
#define RICHTUNG_B	1
#define RICHTUNG_C	2
#define RICHTUNG_D	3

// Seite 1

#define STEPPERPORT_1	PORTC
#define STEPPERDDR_1    DDRC
#define STEPPERPIN_1    PINC

#define MA_STEP         0
#define MA_RI           1
#define MA_EN           2

#define MB_STEP         3
#define MB_RI           4
#define MB_EN           5

#define END_A0          6           // Bit fuer Endanschlag bei A0
#define END_B0          7           // Bit fuer Endanschlag bei A1


// Seite 2

#define STEPPERPORT_2      PORTB
#define STEPPERDDR_2       DDRB
#define STEPPERPIN_2       PINB

#define MC_STEP            0           // PIN auf Stepperport 2
#define MC_RI              1
#define MC_EN              2

#define MD_STEP            3           // PIN auf Stepperport 2
#define MD_RI              4
#define MD_EN              5

#define END_C0             6           // Anschlagstatus:  Bit fuer Endanschlag bei C0
#define END_D0             7           // Anschlagstatus:  Bit fuer Endanschlag bei D0


#define HALT_PIN           0

#define COUNT_A				0 // 4		// Motorstatus:   Schritte von Motor A zaehlen
#define COUNT_B				1 // 5		// Motorstatus:   Schritte von Motor B zaehlen

#define COUNT_C				2 // 4		// Motorstatus:   Schritte von Motor C zaehlen
#define COUNT_D				3 // 5		// Motorstatus:   Schritte von Motor D zaehlen



#define DC                  7    // DC ON: HI
#define STROM               4    // Stepperstrom Generell ON: LO

#define GO_HOME           7     // Bit fuer befehl beginn home auf cncstatus
#define DC_DIVIDER         1      // teilt die pwm-Frequenz in ISR

#define USB_DATENBREITE 32

uint16_t taste = 0xFF;
uint8_t tastencounter = 0;
uint8_t TastaturCount=0;
uint8_t Taste=0;
uint8_t analogtastaturstatus = 0;
#define TASTE_OFF  0
#define TASTE_ON  1

uint16_t TastenStatus=0;
uint16_t Tastenprellen=0x1F;

volatile uint8_t           cncstatus=0x00;

volatile uint8_t           usbstatus=0x00;
static volatile uint8_t    motorstatus=0x00;
volatile uint8_t           anschlagstatus=0x00;
volatile uint8_t           anschlagcounter=0;

volatile uint8_t status=0;

volatile uint8_t           PWM=0;
static volatile uint8_t    pwmposition=0;
static volatile uint8_t    pwmdivider=0;

// CNC

volatile uint16_t CounterA=0;			// Zaehler fuer Delay von Motor A 
volatile uint16_t CounterB=0;			// Zaehler fuer Delay von Motor B
volatile uint16_t CounterC=0;			// Zaehler fuer Delay von Motor C 
volatile uint16_t CounterD=0;			// Zaehler fuer Delay von Motor D

volatile uint16_t DelayA=24;			// Delay von Motor A 
volatile uint16_t DelayB=24;			// Delay von Motor B 
volatile uint16_t DelayC=24;			// Delay von Motor C 
volatile uint16_t DelayD=24;			// Delay von Motor D 

volatile uint16_t StepCounterA=0;	// Zaehler fuer Schritte von Motor A 
volatile uint16_t StepCounterB=0;	// Zaehler fuer Schritte von Motor B
volatile uint16_t StepCounterC=0;	// Zaehler fuer Schritte von Motor C 
volatile uint16_t StepCounterD=0;	// Zaehler fuer Schritte von Motor D

volatile uint8_t richtung=0;

volatile uint8_t homestatus=0;

volatile uint8_t           timerstatus=0;


uint8_t motorsteps = 48;
uint8_t micro = 1;
volatile uint8_t           bresenhamstatus=0x00; // relevanter motor, in Abschnittladen:bres gesetzt

#define BRES_MOTORA  0
#define BRES_MOTORB  1
#define BRES_MOTORC  2
#define BRES_MOTORD  3

volatile uint16_t           bresenham_errAB = 0; // error A,B
volatile uint16_t           bresenham_e2AB = 0; // check A,B

volatile uint16_t           bresenham_errCD = 0;
volatile uint16_t           bresenham_e2CD = 0;

volatile uint16_t           StepStartA = 0; // startwert am Anfang des Abschnittes
volatile uint16_t           StepStartC = 0;

// Seite A
volatile int16_t xA, yA, tA, dxA, dyA, incxA, incyA, pdxA, pdyA, ddxA, ddyA, deltaslowdirectionA, deltafastdirectionA, errA;

volatile uint16_t deltafastdelayA = 0; // aktueller delay 
volatile uint16_t bres_delayA = 0; // steps fuer fastdirection
volatile uint16_t bres_counterA = 0; // zaehler fuer fastdirection

//Seite B
volatile int16_t xB, yB, tB, dxB, dyB, incxB, incyB, pdxB, pdyB, ddxB, ddyB, deltaslowdirectionB, deltafastdirectionB, errB;

volatile uint16_t deltafastdelayB = 0; // aktueller delay 
volatile uint16_t bres_delayB = 0; // steps fuer fastdirection
volatile uint16_t bres_counterB = 0; // zaehler fuer fastdirection

volatile uint16_t          ramptimerintervall = TIMERINTERVALL;

volatile uint8_t           rampstatus=0;
//volatile uint8_t           RampZeit = RAMPZEIT;
//volatile uint8_t           RampFaktor = RAMPFAKTOR;
volatile uint32_t          rampstepstart=0; // Stepcounter am Anfang
//volatile uint32_t          ramptimercounter=0;  // laufender counter  fuer Rampanpassung
//volatile uint32_t          //ramptimerdelay = 100;  // Takt fuer Rampanpassung
uint8_t                    rampschritt = 2;
volatile uint16_t          rampbreite = 0;  // anzahl Schritte der Ramp. Wird beim Start bestimmt und fuer das Ende verwendet

volatile uint32_t          rampendstep = 0; // Beginn der Endramp. Wird in Abschnittladen bestimmt
volatile uint16_t          timerintervall = TIMERINTERVALL;
volatile uint16_t          timerintervall_SLOW = 0; // Intervall klein
volatile uint16_t          timerintervall_FAST = 0; // Intervall gross

uint8_t richtungstatus = 0;
uint8_t oldrichtungstatus = 0;
#define AXNEG  0
#define AYNEG  1
#define BXNEG  4
#define BYNEG  5

int16_t lastdax = 0; // letzte Werte fuer schritte x, y. Fuer berechnung gradient
int16_t lastday = 0;


uint16_t errpos = 0;
// bresenham end
volatile uint16_t timer3Counter=0; 

// https://www.cprogramcoding.com/p/box-sizing-border-box_549.html
int maxx(int num1, int num2)
{
    return (num1 > num2 ) ? num1 : num2;
}
void startTimer3()
{
   timerstatus |= (1<<TIMER_ON);
   
   // Timer3: Normal mode with overflow interrupt
   TCNT3 = 0;
   
   // Normal mode
   TCCR3A = 0x00;
   
   // Prescaler: Fcpu/8 (CS31 = 1, CS30 = 0)
   TCCR3B = (1 << CS31);
   
   // Enable overflow interrupt
   TIMSK3 |= (1 << TOIE3);
   
   sei();
}

void stopTimer3(void)
{
   TCCR3B = 0;
   timerstatus &= ~(1<<TIMER_ON);
}

void slaveinit(void)
{
	//OSZIPORTDDR |= (1<<PULS);	// Output
	//OSZIPORT |= (1<<PULS);		// HI
	
	LOOPLEDDDR |=(1<<LOOPLED);
	LOOPLEDPORT |= (1<<LOOPLED);	// HI

	STEPPERDDR_1 |= (1<<MA_STEP);
	STEPPERPORT_1 |= (1<<MA_STEP);	// HI
	
	STEPPERDDR_1 |= (1 << MA_RI);
	STEPPERPORT_1 |= (1 << MA_RI);	// HI
   
	STEPPERDDR_1 |= (1 << MA_EN);
	STEPPERPORT_1 |= (1 << MA_EN);	// HI
	
	STEPPERDDR_1 |= (1 << MB_STEP);
	STEPPERPORT_1 |= (1 << MB_STEP); // HI
	
	STEPPERDDR_1 |= (1 << MB_RI);
	STEPPERPORT_1 |= (1 << MB_RI);	// HI
	
	STEPPERDDR_1 |= (1 << MB_EN);
	STEPPERPORT_1 |= (1 << MB_EN); // LO
   
   //Seite 2
	STEPPERDDR_2 |= (1<<MC_STEP);
	STEPPERPORT_2 |= (1<<MC_STEP);	// HI
	
	STEPPERDDR_2 |= (1 << MC_RI);
	STEPPERPORT_1 |= (1 << MC_RI);	// HI
   
	STEPPERDDR_2 |= (1 << MC_EN);
	STEPPERPORT_2 |= (1 << MC_EN);	// HI
	
	STEPPERDDR_2 |= (1 << MD_STEP);
	STEPPERPORT_2 |= (1 << MD_STEP); // HI
	
	STEPPERDDR_2 |= (1 << MD_RI);
	STEPPERPORT_2 |= (1 << MD_RI);	// HI
	
	STEPPERDDR_2 |= (1 << MD_EN);
   STEPPERPORT_2 |= (1 << MD_EN);	// HI
   
	
	//Pin 0 von   als Ausgang fuer OSZI
	OSZIPORTDDR |= (1<<OSZI_PULS_A);	//Pin 0 von  als Ausgang fuer LED TWI
    OSZIPORT |= (1<<OSZI_PULS_A);		// HI
	
    OSZIPORTDDR |= (1<<OSZI_PULS_B);		//Pin 1 von  als Ausgang fuer LED TWI
    OSZIPORT |= (1<<OSZI_PULS_B);		//Pin   von   als Ausgang fuer OSZI
	

	TASTENDDR &= ~(1<<TASTE0);	//Bit 0 von PORT B als Eingang für Taste 0
	TASTENPORT |= (1<<TASTE0);	//Pull-up

//	DDRB &= ~(1<<PORTB1);	//Bit 1 von PORT B als Eingang für Taste 1
//	PORTB |= (1<<PORTB1);	//Pull-up
	

	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);		//Pin 4 von PORT D als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT D als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT D als Ausgang fuer LCD

	
   DDRD |= (1<<PORTD6);
  PORTD |= (1<<PORTD6);
   
   
   // Anschlaege
   
   STEPPERDDR_1 &= ~(1<<END_A0_PIN);			//	Eingang für Endanschlag A0
	STEPPERPORT_1 |= (1<<END_A0_PIN);			// Pull-up
   
	STEPPERDDR_1 &= ~(1<<END_B0_PIN);			//	Eingang für Endanschlag B0
	STEPPERPORT_1 |= (1<<END_B0_PIN);			// Pull-up
   
   
   STEPPERDDR_2 &= ~(1<<END_C0_PIN);			//	Eingang für Endanschlag C0
	STEPPERPORT_2 |= (1<<END_C0_PIN);			// Pull-up
   
   STEPPERDDR_2 &= ~(1<<END_D0_PIN);			//	Eingang für Endanschlag D0
	STEPPERPORT_2 |= (1<<END_D0_PIN);			// Pull-up
   

   
   
   CMD_DDR |= (1<<DC);                       // DC-PWM-Ausgang
   CMD_PORT &= ~(1<<DC);                      // LO
   
   CMD_DDR |= (1<<STROM);                    // Stepperstrom-Ausgang, Active HI
   CMD_PORT |= (1<<STROM);                   // HI
}

ISR (TIMER3_OVF_vect) 
{ 
	timer3Counter +=1;
   
   if (PWM) // Draht soll heiss sein. 
   {
   }
   else
   {
      pwmposition =0;
   }

	if (timer3Counter >= 14) 
	{
       
      if (CounterA)  CounterA-=1;
      if (CounterB) 	CounterB-=1;
      if (CounterC)  CounterC-=1;
      if (CounterD)  CounterD-=1;
      
      
      if (timerstatus & (1<<TIMER_ON))
      {
         // OSZI_A_LO(); // 100us
         
         if (bres_delayA)
         {
            bres_delayA-=1;
         }
         
         if (bres_delayB)
         {
            bres_delayB-=1;
         }
         
      
         
      } 
     
      
      
      if (PWM)
      {
         pwmposition ++;
      }
      else
      {
         pwmposition =0;
      }

		timer3Counter = 0; 
        //OSZIBTOG ;
	} 
	TCNT3 = 10;							// ergibt 2 kHz fuer Timertakt
}

void setup() 
{
  // Initialize LED pin as output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize Raw HID
  rawhid_init();
  
  delay(1000); // Wait for USB enumeration
}

void loop() {
  // Process incoming Raw HID data
  process_rawhid_data();
  
  // Simple heartbeat LED blink
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(900);
}