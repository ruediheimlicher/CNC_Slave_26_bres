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

#include "lcd.c"

#include "adc.c"

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

uint16_t loopcount0=0;
uint8_t loopcount1=0;

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

volatile uint16_t Tastenwert=0;
uint16_t tastaturcounter = 0;

// https://www.cprogramcoding.com/p/box-sizing-border-box_549.html
int maxx(int num1, int num2)
{
    return (num1 > num2 ) ? num1 : num2;
}
void startTimer3(void)
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
// MARK:  AbschnittLaden_bres
uint8_t  AbschnittLaden_bres(const uint8_t* AbschnittDaten) // 22us
{
   stopTimer3();
 //  lcd_gotoxy(15,0);
 //  lcd_puts("    ");
   
   uint8_t returnwert=0;

   /*         
    Reihenfolge der Daten:
    0    schritteax lb
    1    schritteax hb
    2    schritteay lb
    3    schritteay hb
    
    4    delayax lb
    5    delayax hb
    6    delayay lb
    7    delayay hb
    
    8    schrittebx lb
    9    schrittebx hb
    10    schritteby lb
    11    schritteby hb
    
    12    delaybx lb
    13    delaybx hb
    14    delayby lb
    15    delayby hb
    
    
    16   (8)    code
    17   (9)    position // Beschreibung der Lage im Schnittpolygon:first, last, ...
    18   (10)   indexh     // Nummer des Abschnitts
    19   (11)   indexl   
    
    20     pwm
    
    21   motorstatus // relevanter Motor fuer Abschnitt
    
    22   zoomfaktor
    
    25   steps
    26   micro
    
    */         
   
   motorsteps = AbschnittDaten[25];
   
   micro = AbschnittDaten[26];
   
   uint16_t index = (AbschnittDaten[18] << 8) | AbschnittDaten[19];
   
   if (AbschnittDaten[35] == 1)
   {
      // Serial.printf("+++ +++ +++ \t\t\t index: %d AbschnittLaden_bres WENDEPUNKT \n",index);
      rampstatus |=(1<<RAMPOKBIT);
   }
   
   // pwm-rate
   PWM = AbschnittDaten[20];
   //// Serial.printf("AbschnittLaden_4M steps: %d micro: %d PWM: %d\n",steps,micro,PWM);
   //// Serial.printf("AbschnittLaden_bres start \n");
   //**   analogWrite(DC_PWM, PWM);
   
   //// Serial.printf("AbschnittLaden_bres AbschnittDaten eingang index: %d\n", index);
   
   
   /*
   for(uint8_t i=0;i<27;i++) // 5 us ohne printf, 10ms mit printf
   { 
      // Serial.printf("%d \t",AbschnittDaten[i]);
   }
   // Serial.printf("\n");
   
    //  // Serial.printf("AbschnittLaden_4M steps: %d micro: %d\n",motorsteps,micro);
   
   //   lage = AbschnittDaten[9]; // Start: 1, innerhalb: 0, Ende: 2
    */
   
   int lage = 0;
   lage = AbschnittDaten[17]; // Start: 1, innerhalb: 0, Ende: 2
  // // Serial.printf("******* *********   AbschnittLaden_bres lage: %d\n",lage);
  // // Serial.printf("AbschnittLaden_bres lage: %d\n",lage);
   if (lage & 0x01) // erstes Element
   {
      returnwert=1;
   }
   if (lage & 0x02) // letztes Element
   {
      returnwert=2;
   }
   
   richtung=0;
   
   // Motor A
   //digitalWriteFast(MA_EN,LOW); // Pololu ON
   STEPPERPORT_1 &= ~(1<<MA_EN); // Pololu ON
   uint8_t dataL=0;
   uint8_t dataH=0;
   
   uint8_t delayL=0;
   uint8_t delayH=0;
   
   dataL=AbschnittDaten[0];
   dataH=AbschnittDaten[1];
   
   //lcd_gotoxy(17,0);
   int8_t vz = 1;
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_A); // Rueckwarts
      //digitalWriteFast(MA_RI, LOW); // PIN fuer Treiber stellen
      STEPPERPORT_1 &= ~(1<< MA_RI); // PIN fuer Treiber stellen
      vz = -1;
      //lcd_putc('r');
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_A);
      //digitalWriteFast(MA_RI, HIGH);
      STEPPERPORT_1 |= (1<< MA_RI);
      //lcd_putc('v');   // Vorwaerts
   }
   
   dataH &= (0x7F); // bit 8 entfernen
   StepCounterA = dataL | (dataH << 8);      //    

   StepCounterA *= micro;
   StepStartA = StepCounterA;
      
   delayL=AbschnittDaten[4];
   delayH=AbschnittDaten[5];
   
   DelayA = delayL | (delayH << 8);
   
   CounterA = DelayA;
   
   // Motor B
   STEPPERPORT_1 &= ~(1<<MB_EN);   // Pololu ON
   
   dataL=AbschnittDaten[2];
   dataH=AbschnittDaten[3];
   //lcd_gotoxy(19,1);
   vz = 1;
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_B); // Rueckwarts
      //digitalWriteFast(MB_RI, LOW);      //lcd_putc('r');
      STEPPERPORT_1 &= ~(1<< MB_RI);
      vz = -1;
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_B);
      STEPPERPORT_1 |= (1<< MB_RI);
   }
   
   dataH &= (0x7F);
    
   StepCounterB = dataL | (dataH <<8);
   
   StepCounterB *= micro;
   
   
    DelayB = (AbschnittDaten[7]<<8) | AbschnittDaten[6];
   
   
   // Serial.printf("\nAbschnittLaden_bres index: %d StepCounterA  : %d DelayA: %d StepCounterB: %d DelayB: %d\n",index,StepCounterA, DelayA, StepCounterB, DelayB);

   
   CounterB = DelayB;
   
   
    // Motor C
   //digitalWriteFast(MC_EN,LOW);    // Pololu ON
   STEPPERPORT_2 &= ~(1<<MC_EN); // Pololu ON
   //CounterC=0;
   dataL=0;
   dataH=0;
   
   delayL=0;
   delayH=0;
   
   dataL=AbschnittDaten[8];
   dataH=AbschnittDaten[9];
   //// Serial.printf("AbschnittLaden_4M C datah: %d\n",dataH);
   //richtung=0;
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_C); // Rueckwarts
      //digitalWriteFast(MC_RI, LOW);
      STEPPERPORT_2 &= ~(1<< MC_RI);
      //// Serial.printf("AbschnittLaden_4M C negativ\n");
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_C);
      //digitalWriteFast(MA_RI, HIGH);
      STEPPERPORT_2 |= (1<< MC_RI);
      //// Serial.printf("AbschnittLaden_4M C positiv\n");
   }
   
   dataH &= (0x7F);
//   StepCounterC = dataH;      // HByte
//   StepCounterC <<= 8;      // shift 8
//   StepCounterC += dataL;   // +LByte
   
   StepCounterC = dataL | (dataH << 8);
   StepCounterC  *= micro;
   
   StepStartC = StepCounterC;
   delayL=AbschnittDaten[12];
   delayH=AbschnittDaten[13];
   
//   DelayC = delayH;
//   DelayC <<=8;
//   DelayC += delayL;
   
   DelayC = delayL | (delayH <<8);
   
   CounterC = DelayC;

   // Motor D
   STEPPERPORT_2 &= ~(1<<MD_EN); // Pololu ON
   dataL=0;
   dataH=0;
   
   delayL = 0;
   delayH = 0;
   
   dataL = AbschnittDaten[10];
   dataH = AbschnittDaten[11];
   //// Serial.printf("AbschnittLaden_4M D datah: %d\n",dataH);
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_D); // Rueckwarts
      STEPPERPORT_2 &= ~(1<<MD_RI); // Rueckwarts
      //// Serial.printf("AbschnittLaden_4M D negativ\n");
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_D);
      STEPPERPORT_2 |= (1<< MD_RI);
      //// Serial.printf("AbschnittLaden_4M D positiv\n");
   }
   
   dataH &= (0x7F);
   
   StepCounterD = dataL | (dataH << 8); 
   StepCounterD  *= micro;
   
   delayL=AbschnittDaten[14];
   delayH=AbschnittDaten[15];
   
   DelayD = delayL | (delayH <<8);
   
   //// Serial.printf("AbschnittLaden_4M StepCounterD: %d DelayD: %d\n",StepCounterD,DelayD);
   CounterD = DelayD;
   
   //  ****
   //  Bresenham
   //  ***
   //// Serial.printf("AbschnittLaden_bres vor bresenham: StepCounterA: %d StepCounterB: %d\n",StepCounterA,StepCounterB);
   deltafastdirectionA = 0;
   deltaslowdirectionA = 0;
   deltafastdirectionB = 0;
   deltaslowdirectionB = 0;
   deltafastdelayA = 0;
   deltafastdelayB = 0;

   bresenhamstatus = 0;
   // bresenham Seite A
   
   // relevanten Motor setzen
   if (StepCounterA > StepCounterB)
   {
      bresenhamstatus |= (1<<BRES_MOTORA);
      //
      pdxA = 1;
      pdyA = 0;
      ddxA = 1;
      ddyA = 1;
      deltaslowdirectionA = StepCounterB;
      deltafastdirectionA = StepCounterA;
      deltafastdelayA = DelayA;
 //     // Serial.printf("AbschnittLaden_bres  A > B\n");
   }
   else
   {
      bresenhamstatus |= (1<<BRES_MOTORB);
      //
      pdxA = 0;
      pdyA = 1;
      ddxA = 1;
      ddyA = 1;
      deltaslowdirectionA = StepCounterA;
      deltafastdirectionA = StepCounterB;
      deltafastdelayA = DelayB;
 //     // Serial.printf("AbschnittLaden_bres  A < B\n");
   }
   // aktuelle Werte einsetzen
   bres_delayA = deltafastdelayA; // aktueller delay in fastdir
   bres_counterA = deltafastdirectionA; // aktueller counter fuer steps
   
   if(rampstatus & (1<<RAMPOKBIT))
   {
      // Serial.printf("AbschnittLaden_bres index: %d set RAMPSTARTBIT\n",index);
      rampstatus |= (1<<RAMPSTARTBIT);
      errpos = 0;
      ramptimerintervall += (ramptimerintervall/4*3);
      
      //delayTimer.update(ramptimerintervall);
   }
   
   xA = StepCounterA; // 
   yA = StepCounterB;

   errA = deltafastdirectionA/2;
   
  // // Serial.printf("AbschnittLaden_bres deltafastdirectionA: %d deltaslowdirectionA: %d  deltafastdelayA: %d errA: %d bres_counterA: %d bres_delayA: %d\n",deltafastdirectionA,deltaslowdirectionA, deltafastdelayA,errA,bres_counterA,bres_delayA);

   // bresenham Seite B
   
    
   
   // relevanten Motor setzen
   if (StepCounterC > StepCounterD)
   {
      bresenhamstatus |= (1<<BRES_MOTORC);
      //
      pdxB = 1;
      pdyB = 0;
      ddxB = 1;
      ddyB = 1;
      deltaslowdirectionB = StepCounterD;
      deltafastdirectionB = StepCounterC;
      deltafastdelayB = DelayC;
      //// Serial.printf("AbschnittLaden_bres  C > D\n");
   }
   else
   {
      bresenhamstatus |= (1<<BRES_MOTORD);
      //
      pdxB = 0;
      pdyB = 1;
      ddxB = 1;
      ddyB = 1;
      deltaslowdirectionB = StepCounterC;
      deltafastdirectionB = StepCounterD;
      deltafastdelayB = DelayD;
      //// Serial.printf("AbschnittLaden_bres  C < D\n");
   }
   // aktuelle Werte einsetzen
   bres_delayB = deltafastdelayB; // aktueller delay in fastdir
   bres_counterB = deltafastdirectionB; // aktueller counter fuer steps
   
   xB = StepCounterC; // 
   yB = StepCounterD;

   errB = deltafastdirectionB/2;
   
     {
   
   timerintervall_FAST = TIMERINTERVALL;
   //  OSZI_B_LO();
   }
   
   
   // motorstatus: welcher Motor ist relevant
   motorstatus = AbschnittDaten[21];
   
   // richtung change
#pragma mark Richtung change
   
  // rampstatus |=(1<<RAMPOKBIT);

   
   startTimer3();
   
   //// Serial.printf("\nAbschnittLaden_bres end aktuellelage: %d \n",returnwert);
     return returnwert;
 
}



void AnschlagVonMotor(const uint8_t motor) // Schlitten ist am Anschlag
{
   //NSLog(@"AnschlagVonMotor: %d anschlagstatus am Beginn: %d",motor, anschlagstatus);
   if (richtung & (1<<(RICHTUNG_A + motor))) // Richtung war auf Anschlag A0 zu         
   {
      anschlagcounter ++;
       // MARK: END_A0 + motor
      if (!(anschlagstatus &(1<< (END_A0 + motor)))) // Bit noch nicht gesetzt
      {
         cli();
         PWM = 0;
         lcd_gotoxy(12,2);
         lcd_putc('A' + motor);
         lcd_putc('0');
         anschlagstatus |= (1<< (END_A0 + motor));      // Bit fuer Anschlag A0+motor setzen
         //anschlagstatus |= (1<< (END_A0 + motor + 4)); 
   
        // NSLog(@"anschlagstatus gesetzt: %d cncstatus: %d" anschlagstatus, cncstatus);
         //cncstatus |=  (1<<GO_HOME);
         if (cncstatus & (1<<GO_HOME)) // nur eigene Seite abstellen
         {
   // ********************************* Start HOME *****************
            // Zuerst kommt der Schlitten am Anschalg A oder C an
            
            lcd_gotoxy(15,0);
            lcd_puts("home");
           // Zuerst horizonal auf Anschlag
            switch (motor) // Stepperport 1
            {
               case 0:
               {
                  
               }
                                    
            }//switch motor
            //lcd_gotoxy(10,1);
            //lcd_putc('L');
            //lcd_putint2(ladeposition);
            
            sendbuffer[0]=0xB5 + motor; // HOME Ankunft melden
            cncstatus |= (1<<motor); 
            
             
            if (motor<2) // Stepperport 1
            {
               //lcd_gotoxy(0,2);
               
               //lcd_puts("P1 M");
               //lcd_putint1(motor);

               STEPPERPORT_1 |= (1<<(MA_EN + motor)); // Motor 0 ODER 1 OFF // andere Richtung kommt anschliessend von master
               
               if (anschlagstatus &(1<< END_A0)) // Anschlag von Motor A               
               {
                  //lcd_gotoxy(6,3);
                  //lcd_puts("A0");
                  //StepCounterA=0; 
                  //StepCounterB=0; 
                  //   deltafastdirectionB = 0;
                  //  deltaslowdirectionB = 0;
               }
                  
                  if (anschlagstatus &(1<< END_B0)) // Anschlag von Motor B, NACH Motor A
                  {
  
                     //lcd_gotoxy(8,3);
                     //lcd_puts("B0");
                     //StepCounterB=0; 
                  }
               
               // 
               //StepCounterB=0; 
                //             CounterA=0xFFFF;
                //             CounterB=0xFFFF;
               
            }
            else // Stepperport 2
            {
               //lcd_gotoxy(0,3);
               //lcd_puts("P2 M");
               //lcd_putint1(motor);
               

               STEPPERPORT_2 |= (1<<(MA_EN + motor));     // Motor 2,3 OFF
               //STEPPERPORT_1 |= (1<<(MA_EN + motor - 2)); // Paralleler Motor 0,1 OFF
               
               if (anschlagstatus &(1<< END_C0)) // Anschlag von Motor C               
               {
                  //lcd_gotoxy(6,3);
                  //lcd_puts("C0");
                  //StepCounterC=0; 
                  //StepCounterD=0;
                 
               }
                  
   
                  if (anschlagstatus &(1<< END_D0)) // Anschlag von Motor D, NACH Motor C
                  {
                     //lcd_gotoxy(8,3);
                     //lcd_puts("D0");
                     StepCounterD=0; 
                  }
               
              
              // StepCounterD=0;
               //               CounterC=0xFFFF;
               //               CounterD=0xFFFF;
               
            } // end Stepperport 2
         
    //        cncstatus &= ~(1<<GO_HOME);
            //ladeposition=0;
    //        AbschnittCounter++;
            //sendbuffer[0]=0xEA;
            
    //        lcd_putc('C');
    //        lcd_putint(cncstatus);

 //           lcd_puthex(STEPPERPIN_1);
//            lcd_puthex(STEPPERPIN_2);
            
            sendbuffer[7]=abschnittnummer; // lo
            sendbuffer[8]=ladeposition;
            sendbuffer[22] = cncstatus;
            sendbuffer[19] = anschlagstatus;
            
            sendbuffer[23] = (StepCounterA & 0xFF0)>>8;
            sendbuffer[24] = StepCounterA & 0x00FF;
            sendbuffer[25] = (StepCounterB & 0xFF0)>>8;
            sendbuffer[26] = StepCounterB & 0x00FF;
            sendbuffer[27] = (StepCounterC & 0xFF0)>>8;
            sendbuffer[28] = StepCounterC & 0x00FF;
            sendbuffer[29] = (StepCounterD & 0xFF0)>>8;
            sendbuffer[30] = StepCounterD & 0x00FF;
            lcd_gotoxy(0,0);
            lcd_puts("code ");
            lcd_gotoxy(6+motor,0);
            lcd_puthex(sendbuffer[0]);
 //           usb_rawhid_send((void*)sendbuffer, 50); // 220518 diff
            sei();
            
  //          cncstatus &= ~(1<<GO_HOME);
            
            
            
    //        richtung &= ~(1<<(RICHTUNG_A + motor)); // Richtung umschalten // 220518 diff
// ********************************* End HOME *****************
         } // end HOME
         
         else           // beide Seiten abstellen, Vorgang unterbrechen
         {    
            lcd_gotoxy(10,0);
            lcd_puts("both");
            cncstatus=0;
            sendbuffer[0]=0xA5 + motor;
            
            if (motor<2) // Stepperport 1
            {
               deltafastdirectionA = 0;
               deltafastdirectionB = 0;
               deltaslowdirectionA = 0;
               deltaslowdirectionB = 0;
               
               
               STEPPERPORT_1 |= (1<<(MA_EN + motor));     // Motor 0,1 OFF
               STEPPERPORT_2 |= (1<<(MC_EN + motor)); // Paralleler Motor 2,3 OFF
               
               
            } // end Stepperport 1
            else // Stepperport 2
            {
               deltafastdirectionA = 0;
               deltafastdirectionB = 0;
               deltaslowdirectionA = 0;
               deltaslowdirectionB = 0;
              
               
               STEPPERPORT_2 |= (1<<(MA_EN + motor));     // Motor 2,3 OFF
               STEPPERPORT_1 |= (1<<(MC_EN + motor )); // Paralleler Motor 0,1 OFF
            }
            
            // Alles abstellen
            /*
            StepCounterA=0;
            StepCounterB=0;
            StepCounterC=0;
            StepCounterD=0;
            */
            /*
            CounterA = 0;
            CounterB = 0;
            CounterC = 0;
            CounterD = 0;
            */
            ladeposition=0;
            motorstatus=0;
             
            sendbuffer[5]=abschnittnummer;
            sendbuffer[6]=ladeposition;
            sendbuffer[22] = cncstatus;
            lcd_gotoxy(0,0);
            lcd_puts("code ");
            lcd_gotoxy(6+motor,0);
            lcd_puthex(sendbuffer[0]);

            rawhid_send((void*)sendbuffer, 32,50);
            sei();
             richtung &= ~(1<<(RICHTUNG_A + motor)); // Richtung umschalten
            
         } // both
         
         sei();
      } // END_A0 + motor
      else
      {
         
      }
      
   } // richtung war auf Anschlag zu
   /*
   else  // richtung ist von Anschlag weg
   {
       if ((anschlagstatus &(1<< (END_A0 + motor))))
      {
         anschlagstatus &= ~(1<< (END_A0 + motor)); // Bit fuer Anschlag X0 zuruecksetzen
         lcd_gotoxy(12,2);
         lcd_putc('x');
         lcd_putc('x');
      }
      else
      {
      }
      
   }
   */
}



void setup() 
{
  int8_t r;

   uint16_t count=0;

   // set for 16 MHz clock
   CPU_PRESCALE(0);
  // Initialize LED pin as output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize Raw HID
  rawhid_init();
  
  _delay_ms(1000); // Wait for USB enumeration
  
  lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

   lcd_puts("Guten Tag\0");
   _delay_ms(1000);
   lcd_cls();
   //lcd_puts("READY\0");
slaveinit();
initADC(2);
PWM = 0;
   
     lcd_clr_line(0);

}

void loop() {
  // Process incoming Raw HID data
  process_rawhid_data();
  
  tastaturcounter++;
      if (tastaturcounter > 0xFFF)
      {
         tastaturcounter = 0;
         Tastenwert=readKanal(TASTATURPIN)>>2;
         
      }
      //Blinkanzeige
      loopcount0+=1;
      if (loopcount0==0x8FFF)
      {
         loopcount0=0;
         loopcount1+=1;
         LOOPLEDPORT ^=(1<<LOOPLED);
         PORTD ^= (1<<PORTD6);
      }// if loopcount


  // Simple heartbeat LED blink
  
}