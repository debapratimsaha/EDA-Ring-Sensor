/*  CIT Sensor Ring Project
**  Code by : DebaPSaha 
**  Date    : 5/22/14
**  Ver     : 1.0     
*/    

// ---------- Headers -------------------------	//
#include <avr/io.h>                    								// Adds useful constants
#include <util/delay.h>                								// Adds delay_ms and delay_us functions
#include <SoftwareSerial.h>            							        // Library for software serial, ATtiny doesn't have hardware serial
#include "Arduino.h"                   								// Useful constants
	
// ------------- Definitions ------------------	//
#define DEBUG_MODE          	false      							// Serial prints are sent to the prompt
#define F_CPU               	8000000    							// delay.h uses this value, we have set the cpu freq to be 8MHz
#define INITPULSETIME       	30         							// LED Pulse half-period during initialization 
#define MAX_PWIDTH          	20         							// LED PWM max pulse width
#define SIZEOF_DATA_ARRAY   	64        							// Data_Array is the array containing time series
#define SCL_LOOPCOUNTER         4000                                                            // Assuming 250uS for each loop, 4000 sets the SCL polling at 1sec
//#define LEDTRANSITION_LOOPDELAY (int)(500*1000)/(MAX_PWIDTH*21);


#if DEBUG_MODE
// Port Definitions
#define rxPin 3
#define txPin 4
// Instantiate serial port
SoftwareSerial mySerial(rxPin, txPin);
#else
// Define the LED Pins    
#define LED_R    3              // Pin 3 -- Red LED
#define LED_G    4              // Pin 4 -- Green LED
#endif
#define LED_B    0              // Pin 0 -- Blue LED

// Define ADC input 
#define GSR_SIG  1              // Pin 2 -- GSR Signal | 1 = Analog Pin 1


// -------------- Global Vars ------------------------//
int TH[5];                                            // This var stores the region of operation of LED wrt GSR values
int gsrData[SIZEOF_DATA_ARRAY];                       // This array stores the values of GSR data to compute SCL
float tonicSCL;
int loopCount=SCL_LOOPCOUNTER-1, dataCount=0;
enum ledColor {Red,Green,Blue,Null};
enum stateColor {OnlyRed,OnlyGreen,OnlyBlue,Magenta,Yellow,Cyan};
stateColor currentState,toState=OnlyRed;

//------------- Function Prototypes -------------//
void pulse(int, int);
void stateMachine(int);
void attinySWpwm(int,int);
void setupExtraPWM(void);
void stateTransition(stateColor,stateColor);
void maintainState(stateColor);
float calcSCL(void);
void mapLEDrange(float);

// --------------- Start of Program -----------------------//  

/************** Function setup() **********/
void setup() {  
  // Setup pinmodes
#if DEBUG_MODE  
  mySerial.begin(9600);  
#else
  pinMode(LED_R, OUTPUT);     
  pulse (LED_R,2);
  pinMode(LED_G, OUTPUT);     
  pulse (LED_G,2);
#endif  
  pinMode(LED_B, OUTPUT);     
  pulse (LED_B,2);
  
  pinMode(GSR_SIG, INPUT); 

  // Toggle Pin setup
  pinMode(1,OUTPUT)  ;        
  
  // Setup an Extra PWM 
  setupExtraPWM();
}


//******************* LOOP ***********************//
void loop() {
  int gsrVal = analogRead(GSR_SIG);
  gsrData[dataCount++]=gsrVal;
  if (dataCount==SIZEOF_DATA_ARRAY) dataCount=0;
  
  loopCount++;
  if (loopCount==4000){
    tonicSCL =  calcSCL();
    mapLEDrange(tonicSCL);
    loopCount = 0;
  }

#if DEBUG_MODE
  mySerial.println((int)tonicSCL);
  delay(100);
#else
  stateMachine(gsrVal);
#endif
}


/************** Function mapLEDrange() **********/
void mapLEDrange(float tonicSCL){
// Define thresholds and colors
/*<-- Red -->|| TH0 ||<-- Magenta -->|| TH1 ||<-- Yellow -->| ... 
   ... | TH2 ||<-- Cyan -->|| TH3 ||<-- Green -->|| TH4 ||<-- Blue --> -*/

  //------------ Method I --------------//
//  int stepSize = (int)(1023 - tonicSCL)/6;             //This is the step sie for the LED color ranges
//  for (int i=1;i<6;i++)  TH[i-1] = 1023-i*stepSize;    //Map the LED ranges according to the SCL value  
  
  //------------ Method II -------------//
  int upLim,lowLim;
  if (tonicSCL < 300){
    upLim = 300;  lowLim = 0;
  }else if(tonicSCL < 700){
    upLim = 700;  lowLim = 301;  
  }else {
    upLim = 1023;  lowLim = 701;  
  }
  int stepSize = (int)(upLim - lowLim + 100)/6;           //This is the step sie for the LED color ranges
  for (int i=1;i<6;i++)  TH[i-1] = upLim - i*stepSize;    //Map the LED ranges according to the SCL value  
  
}

/************** Function stateMachine() **********/
void stateMachine(int gsrVal){
#if DEBUG_MODE  
  delay(20);                              // Delay needed for the serial to work   
#else  
//  attinySWpwm(LED_R,50);//map(gsrVal,500,1023,0,400));
//  OCR1B = 10;//map(gsrVal,500,1023,0,250);    // Green LED
//  OCR0A = 10;//map(gsrVal,500,1023,0,10);    // Blue LED

  currentState = toState;                  // Store the previous state
  if (gsrVal>=TH[0]){
    toState=OnlyRed;
  }else if(gsrVal<TH[0] && gsrVal>=TH[1]){
    toState=Magenta;
  }else if(gsrVal<TH[1] && gsrVal>=TH[2]){
    toState=Yellow;
  }else if(gsrVal<TH[2] && gsrVal>=TH[3]){
    toState=Cyan;
  }else if(gsrVal<TH[3] && gsrVal>=TH[4]){
    toState=OnlyGreen;
  }else if(gsrVal<TH[4]){
    toState=OnlyBlue;
  }
  
  // Choose whether to transition or maintain state
  if (toState == currentState){
    maintainState(currentState);
  }else{
    stateTransition(currentState,toState);  
  }
#endif  
}

/************** Function maintainState() **********/
void maintainState(stateColor currentState){
    switch (currentState){
    case OnlyRed:
      attinySWpwm(LED_R,MAX_PWIDTH);
      break;
    case OnlyBlue:
      OCR0A = MAX_PWIDTH;
      break;
    case OnlyGreen:
      OCR1B = MAX_PWIDTH;
      break;
    case Magenta:
      attinySWpwm(LED_R,MAX_PWIDTH);
      OCR0A = MAX_PWIDTH;
      break;
    case Yellow:
      OCR1B = MAX_PWIDTH;
      attinySWpwm(LED_R,MAX_PWIDTH);
      break;
    case Cyan:
      OCR0A = MAX_PWIDTH;
      OCR1B = MAX_PWIDTH;
      break;
    }
}

/************** Function stateTransition() **********/
void stateTransition(stateColor currentState, stateColor toState){  
  ledColor toDec1=Null,toDec2=Null,toInc1=Null,toInc2=Null;
  
  if (toState == OnlyRed){
    switch (currentState){
    case Magenta  :  toDec1=Blue;
    case Yellow   :  toDec1=Green; 
    case Cyan     :  toDec1=Blue; toDec2=Green; toInc1=Red;
    case OnlyGreen:  toDec1=Green; toInc1=Red;
    case OnlyBlue :  toDec1=Blue; toInc1=Red;
    }      
  }else if (toState == Magenta){
    switch (currentState){
    case OnlyRed  :  toInc1=Blue;
    case Yellow   :  toDec1=Green; toInc1=Blue;
    case Cyan     :  toDec1=Green; toInc1=Red;
    case OnlyGreen:  toDec1=Green; toInc1=Red; toInc2=Blue; 
    case OnlyBlue :  toInc1=Red;
    }      
  }else if (toState == Yellow){
    switch (currentState){
    case OnlyRed  :  toInc1=Green; 
    case Magenta  :  toDec1=Blue; toInc1=Green; 
    case Cyan     :  toDec1=Blue; toInc1=Red;
    case OnlyGreen:  toInc1=Red;
    case OnlyBlue :  toDec1=Blue; toInc1=Red; toInc2=Green;
    }      
  }else if (toState == Cyan){
    switch (currentState){
    case OnlyRed  :  toDec1=Red; toInc1=Green;toInc2=Blue;
    case Magenta  :  toDec1=Red; toInc1=Green;
    case Yellow   :  toDec1=Red; toInc1=Blue;
    case OnlyGreen:  toInc1=Blue;
    case OnlyBlue :  toInc1=Green;
    }      
  }else if (toState == OnlyGreen){
    switch (currentState){
    case OnlyRed  :  toDec1=Red; toInc1=Green;
    case Magenta  :  toDec1=Blue; toDec2=Red; toInc1=Green;
    case Yellow   :  toDec1=Red; 
    case Cyan     :  toDec1=Blue;
    case OnlyBlue :  toDec1=Blue;toInc1=Green;
    }      
  }else if (toState == OnlyBlue){
    switch (currentState){
    case OnlyRed  :  toDec1=Red; toInc1=Blue;   
    case Magenta  :  toDec1=Red;
    case Yellow   :  toDec1=Green; toDec2=Red; toInc1=Blue;
    case Cyan     :  toDec1=Green;
    case OnlyGreen:  toDec1=Green;toInc1=Blue;
    }      
  }
  
  //loop for LED increasing and decreasing
  for (int i=0,j=MAX_PWIDTH ; i<=MAX_PWIDTH && j>=0 ; i++,j--){
    switch (toInc1){
    case Red    :  attinySWpwm(LED_R,i+1); break;
    case Blue   :  OCR0A = i; break;
    case Green  :  OCR1B = i; break;
    case Null   :  break;
    }
    switch (toInc2){
    case Red    :  attinySWpwm(LED_R,i+1); break;
    case Blue   :  OCR0A = i; break;
    case Green  :  OCR1B = i; break;
    case Null   :  break;
    }
    switch (toDec1){
    case Red    :  attinySWpwm(LED_R,j+1); if(j==0) digitalWrite(LED_R,LOW); break;
    case Blue   :  OCR0A = j; break;
    case Green  :  OCR1B = j; break;
    case Null   :  break;
    }
    switch (toDec2){
    case Red    :  attinySWpwm(LED_R,j+1); if(j==0) digitalWrite(LED_R,LOW); break;
    case Blue   :  OCR0A = j; break;
    case Green  :  OCR1B = j; break;
    case Null   :  break;
    }
    delay(1000);
  }
}


/************** Function calcSCL() *************/
float calcSCL(){
  uint32_t gsrSCLvalue=0;
  for (int i=0;i<SIZEOF_DATA_ARRAY;i++){
    gsrSCLvalue += gsrData[i];
  }
  return gsrSCLvalue/SIZEOF_DATA_ARRAY;
}

/************** Function attinyPWM() **********/
void attinySWpwm(int pin, int pwmValue){
  digitalWrite(pin,HIGH);
  delayMicroseconds(pwmValue);
  digitalWrite(pin,LOW);
  delayMicroseconds(1023-pwmValue);
}

/************** Function pulse() *************/
void pulse(int pin, int times) {
  do {
    digitalWrite(pin, HIGH);
    delay(INITPULSETIME);
    digitalWrite(pin, LOW);
    delay(INITPULSETIME);
  } 
  while (times--);
}

/*************** Timer/Counter Setup for using Extra PWM **********/
void setupExtraPWM(){
    // This pin configuration setup is copied from URL -http://www.re-innovation.co.uk/web12/index.php/en/blog-75/305-fast-pwm-on-attiny85
    // Set the fast PWM flag
    /*
    Port B Data Direction Register (controls the mode of all pins within port B)
    DDRB is 8 bits: [unused:unused:DDB5:DDB4:DDB3:DDB2:DDB1:DDB0]
    1<<DDB4: sets bit DDB4 (data-direction, port B, pin 4), which puts PB4 (port B, pin 4) in output mode
    1<<DDB1: sets bit DDB1 (data-direction, port B, pin 1), which puts PB1 (port B, pin 1) in output mode
    1<<DDB0: sets bit DDB0 (data-direction, port B, pin 0), which puts PB0 (port B, pin 0) in output mode
    */
//    DDRB = 1<<DDB4 | 1<<DDB1 | 1<<DDB0;

    /*
    Control Register A for Timer/Counter-0 (Timer/Counter-0 is configured using two registers: A and B)
    TCCR0A is 8 bits: [COM0A1:COM0A0:COM0B1:COM0B0:unused:unused:WGM01:WGM00]
    2<<COM0A0: sets bits COM0A0 and COM0A1, which (in Fast PWM mode) clears OC0A on compare-match, and sets OC0A at BOTTOM
    2<<COM0B0: sets bits COM0B0 and COM0B1, which (in Fast PWM mode) clears OC0B on compare-match, and sets OC0B at BOTTOM
    3<<WGM00: sets bits WGM00 and WGM01, which (when combined with WGM02 from TCCR0B below) enables Fast PWM mode
    */
    TCCR0A = 2<<COM0A0 | 2<<COM0B0 | 1<<WGM00;
    
    /*
    Control Register B for Timer/Counter-0 (Timer/Counter-0 is configured using two registers: A and B)
    TCCR0B is 8 bits: [FOC0A:FOC0B:unused:unused:WGM02:CS02:CS01:CS00]
    0<<WGM02: bit WGM02 remains clear, which (when combined with WGM00 and WGM01 from TCCR0A above) enables Fast PWM mode
    1<<CS00: sets bits CS01 (leaving CS01 and CS02 clear), which tells Timer/Counter-0 to not use a prescalar
    */
    TCCR0B = 0<<WGM02 | 1<<CS00;
    
    /*
    Control Register for Timer/Counter-1 (Timer/Counter-1 is configured with just one register: this one)
    TCCR1 is 8 bits: [CTC1:PWM1A:COM1A1:COM1A0:CS13:CS12:CS11:CS10]
    0<<PWM1A: bit PWM1A remains clear, which prevents Timer/Counter-1 from using pin OC1A (which is shared with OC0B)
    0<<COM1A0: bits COM1A0 and COM1A1 remain clear, which also prevents Timer/Counter-1 from using pin OC1A (see PWM1A above)
    1<<CS10: sets bit CS11 which tells Timer/Counter-1  to not use a prescalar
    */
    TCCR1 = 0<<PWM1A | 0<<COM1A0 | 1<<CS10;
    
    /*
    General Control Register for Timer/Counter-1 (this is for Timer/Counter-1 and is a poorly named register)
    GTCCR is 8 bits: [TSM:PWM1B:COM1B1:COM1B0:FOC1B:FOC1A:PSR1:PSR0]
    1<<PWM1B: sets bit PWM1B which enables the use of OC1B (since we disabled using OC1A in TCCR1)
    2<<COM1B0: sets bit COM1B1 and leaves COM1B0 clear, which (when in PWM mode) clears OC1B on compare-match, and sets at BOTTOM
    */
    GTCCR = 1<<PWM1B | 2<<COM1B0;
}
