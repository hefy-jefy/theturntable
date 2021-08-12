//
// 01 modified 14 Dec 14 by Stuart Allen to equalise step times (was 2 then 'pause') > 02
// 02 modified 14 Dec 14 to allow direct writing to io port D
// 04 modified to use array to hold motor phase.
//    functions moveMotor(), stepMotor and releaseMotor() added
// 05 modified 21 Jun 15 to us timer 2 interrupts NOT
// Modified 7 Mar 16 to rotated ANTICLOCK until hall effect sensor detects magnet
//  then CLOCKWISEwise until it is not
//  then CLOCKWISEwise until a zero position
// Added 4 innput switches and 3 output LEDs
// 14 Mar 16 Tidied up code
// 1/10/2018 Modified by Geoff and Lizzie to use the IR Remote from Ardunio instead of buttons
// Also to drive to absolute postions (exits) instead of +\- 15degs
// 1/20/2018 Modified by Geoff to include a servo to drive a crossing gate
// 1/26/2018 Position numbers modified by Geoff for 5:1 gear ratio.
// 2/1/2018 Modified by Geoff for variable backlash
// 2/9/2018 Fixed return to zero. Can't send -ve values to moveMotor


#include "IRremote.h"  //GS
#include <Servo.h> //GS 1/22/2017
Servo myservo; // create servo object to control a servo
int angle = 0; // variable to store the servo position

// constants won't change. They're used here to set pin numbers:
const int hallPin = 12;     // the number of the hall effect sensor pin
const int ledPin =  13;     // the number of the LED pin

int pos = 0;  

int Blue = 3; //motor cable colours (Bn)    NOTE:  Connections are NOT 1 for 1 to driver board (1>3, 2>5, 3>4, 4>6)
int Green = 4; //motor cable colours (Or)
int Yellow = 5; //motor cable colours (R)
int Orange = 6; //motor cable colours (Y)
int receiver = 8; // GS Signal Pin on IR receiver to Arduino Digital Pin 8

int LED1 = 2; // consol LED red.
int LED2 = 11; // consol LED amber.
int LED3 = 13; // consol LED green.
int t1 = 8;     // no of step at speed 1 while accellerating.
int t2 = 16;    // no of steps at speeds 1 and 2 while accellerating
int s1 = 32;    // pause (delay) for speed 3
int s2 =  16;    // pause (delay) for speed 2
int s3 =  8;    // pause (delay) for speed 1

int ANTICLOCK = 1;
int CLOCKWISE = -1;
int BACKLASH = 15;// Backlash for all except exits 1 thro 5 GS 
int DEG10 = 285;  // steps for 10 degrees GS - change from 15 4/20/18
int DEG180 = 5175;  // steps for 180 deg GS
unsigned char motorStep[4] = {0x50,0x48,0x28,0x30}; // (I have no idea what this does! GS)
IRrecv irrecv(receiver);    // create instance of 'irrecv' GS
decode_results results;     // create instance of 'decode_results' GS

int POS1 = 350; // Table of positions for each exit from the turntable (1 through 5)
int POS1BACKCLK = 30; //Backlash to apply in either direction
int POS1BACKANTI = 20; //Backlash can be different for Clock/Anti
int POS2 = -10;  // values determined from calibration/observation of turntable
int POS2BACKCLK = 15;
int POS2BACKANTI = 40;
int POS3 = -363;   
int POS3BACKCLK = 30;
int POS3BACKANTI = 25;
int POS4 = -730;  
int POS4BACKCLK = 35;
int POS4BACKANTI = 30;
int POS5 = -5132; 
int POS5BACKCLK = 30;
int POS5BACKANTI = 30;

// variables will change:
int hallState = 0;          // variable for reading the hall sensor status
int stepPos = 0;
int TTpos =0;
int p =0; // Steps to move from one position to the next

//#define DEBUG 1

void setup()
{
   irrecv.enableIRIn(); // Start the receiver GS
  // initialise input pins.
  pinMode(hallPin, INPUT);
   // initialise output pins.
 myservo.attach(9); // attaches the servo on pin 9 to the servo object
 pinMode(Blue, OUTPUT);
 pinMode(Green, OUTPUT);
 pinMode(Yellow, OUTPUT);
 pinMode(Orange, OUTPUT);
 pinMode(LED1, OUTPUT);
 pinMode(LED2, OUTPUT);
 pinMode(LED3, OUTPUT);
 #ifdef DEBUG
   Serial.begin(57600);
   digitalWrite(LED2, HIGH);
 #endif

 PORTD=(PORTD & 0x87) | motorStep[stepPos];
 delay(8);
 digitalWrite(LED1, HIGH);        // Red LED1 indicates we are moving
 moveWhileHall(ANTICLOCK,16 ,HIGH); // Move ANTICLOCK at speed delay 16 until hall sensor detects turntable magnet
// move to edge of magnet
  moveWhileHall(CLOCKWISE,32,LOW); // move CLOCKWISEwise until magnet no longer detected. GS Speed delay 32
// now move to starting position
   moveMotor(20,ANTICLOCK,32);    // move to zero position speed delay 32
   TTpos=0;                       // set position count to 0
   #ifdef DEBUG
     Serial.write("At position zero\n");
     digitalWrite(LED1, LOW );      // Red LED off
   #endif
} // end of setup

void loop()
{
// main code here, to run repeatedly:
  digitalWrite(LED3,HIGH); //Green LED on - stopped
  if (irrecv.decode(&results)) // have we received an IR signal? GS
  {
    switch(results.value)
    {
        case 0x52A3D41F: // REV button pressed
        digitalWrite(LED3, LOW); //Green LED off -
        moveMotor(DEG10 ,ANTICLOCK,s3);  //10 degrees anticlockwise -changed 4/20/2018 GS
        moveMotor(BACKLASH,CLOCKWISE,s1);
        delay(500);
        break;

        case 0xD7E84B1B: // FWD button pressed
        digitalWrite(LED3, LOW); //Green LED off -
        moveMotor(DEG10 ,CLOCKWISE,s3);  //10 degrees clockwise -changed 4/20/2018 GS
        moveMotor(BACKLASH,ANTICLOCK,s1);
        delay(500);
        break;

        case 0xC101E57B: // 0 button pressed  // Back to position 0
        p = 0;
        if (TTpos>0)
        {
        p = TTpos-0;
        //Serial.println (p);
        digitalWrite(LED3, LOW); //Green LED off -
        moveMotor(p + BACKLASH,CLOCKWISE,s3); //move to position 0 + BACKLASH
        moveMotor(BACKLASH,ANTICLOCK,s1); // Remove Backlash
        delay(500);
        }
        else if (TTpos<0)
        {
        p = 0-TTpos;
        Serial.println (p);
        digitalWrite(LED3, LOW); //Green LED off -
        moveMotor(p + BACKLASH,ANTICLOCK,s3); //move to position 0 + BACKLASH
        moveMotor(BACKLASH,CLOCKWISE,s1);  // Remove Backlash
        delay(500);
        }
        break;
        
        case 0x20FE4DBB: // PLAY/PAUSE button pressed
        digitalWrite(LED3, LOW); //Green LED off -
        moveMotor(DEG180 + BACKLASH,ANTICLOCK,3); //180 degrees clockwise at fastest speed (delay 3)
        moveMotor(BACKLASH,CLOCKWISE,s1);
        delay(500);
        break;

        case 0xA3C8EDDB: // PLUS button pressed
        digitalWrite(LED3, LOW); //Green LED off -
        moveMotor(10,CLOCKWISE,s1); //10 steps clockwise (nudge)
        //moveMotor(BACKLASH,ANTICLOCK,s1); // no backlash
        delay(500);
        break;

        case 0xF076C13B: // MINUS button pressed
        digitalWrite(LED3, LOW); //Green LED off -
        moveMotor(10,ANTICLOCK,s1); //10 steps anticlockwise (nudge)
        //moveMotor(BACKLASH,CLOCKWISE,s1); //no backlash
        delay(500);
        break;

        case 0x9716BE3F: // 1 button pressed
        p = 0;
        if (TTpos>POS1)
        {
        p = TTpos-POS1;
        //Serial.println (p);
        digitalWrite(LED3, LOW); //Green LED off -
        moveMotor(p + POS1BACKCLK,CLOCKWISE,s3); //move to position POS1 + Clockwise Backlash for POS1
        moveMotor(POS1BACKCLK,ANTICLOCK,s1); // Remove Backlash
        delay(500);
        }
        else if (TTpos<POS1)
        {
        p = POS1-TTpos;
        Serial.println (p);
        digitalWrite(LED3, LOW); //Green LED off -
        moveMotor(p + POS1BACKANTI,ANTICLOCK,s3); //move to position POS1 + Anticlock Baklash for POS1
        moveMotor(POS1BACKANTI,CLOCKWISE,s1);  // Remove Backlash
        delay(500);
        }
        break;
        
        case 0x3D9AE3F7: // 2 button pressed
        p = 0;
        if (TTpos>POS2)
        {
        p = TTpos-POS2;
        //Serial.println (p);
        digitalWrite(LED3, LOW); //Green LED off -
        moveMotor(p + POS2BACKCLK,CLOCKWISE,s3); //move to position POS2
        moveMotor(POS2BACKCLK,ANTICLOCK,s1);
        delay(500);
        }
        else if (TTpos<POS1)
        {
        p = POS2-TTpos;
        //Serial.println (p);
        digitalWrite(LED3, LOW); //Green LED off -
        moveMotor(p + POS2BACKANTI,ANTICLOCK,s3); //move to position POS2
        moveMotor(POS2BACKANTI,CLOCKWISE,s1);
        delay(500);
        }
        break;

        case 0x6182021B: // 3 button pressed
        p = 0;
        if (TTpos>POS3)
        {
        p = TTpos-POS3;
        //Serial.println (p);
        digitalWrite(LED3, LOW); //Green LED off -
        moveMotor(p + POS3BACKCLK,CLOCKWISE,s3); //move to position POS3
        moveMotor(POS3BACKCLK,ANTICLOCK,s1);
        delay(500);
        }
        else if (TTpos<POS3)
        {
        p = POS3-TTpos;
        //Serial.println (p);
        digitalWrite(LED3, LOW); //Green LED off -
        moveMotor(p + POS3BACKANTI,ANTICLOCK,s3); //move to position POS3
        moveMotor(POS3BACKANTI,CLOCKWISE,s1);
        delay(500);
        }
        break;

        case 0x8C22657B: // 4 button pressed
        p = 0;
        if (TTpos>POS4)
        {
        p = TTpos-POS4;
        //Serial.println (p);
        digitalWrite(LED3, LOW); //Green LED off -
        moveMotor(p + POS4BACKCLK,CLOCKWISE,s3); //move to position POS4
        moveMotor(POS4BACKCLK,ANTICLOCK,s1);
        delay(500);
        }
        else if (TTpos<POS4)
        {
        p = POS4-TTpos;
        //Serial.println (p);
        digitalWrite(LED3, LOW); //Green LED off -
        moveMotor(p + POS4BACKANTI,ANTICLOCK,s3); //move to position POS4
        moveMotor(POS4BACKANTI,CLOCKWISE,s1);
        delay(500);
        }
        break;

        case 0x488F3CBB: // 5 button pressed
        p = 0;
        if (TTpos>POS5)
        {
        p = TTpos-POS5;
        //Serial.println (p);
        digitalWrite(LED3, LOW); //Green LED off -
        moveMotor(p + POS5BACKCLK,CLOCKWISE,s3); //move to position POS5 (Exit)
        moveMotor(POS5BACKCLK,ANTICLOCK,s1);
        delay(500);
        }
        else if (TTpos<POS5)
        {
        p = POS5-TTpos;
        //Serial.println (p);
        digitalWrite(LED3, LOW); //Green LED off -
        moveMotor(p + POS5BACKANTI,ANTICLOCK,s3); //move to position POS5 (Exit)  
        moveMotor(POS5BACKANTI,CLOCKWISE,s1);
        delay(500);
        }
        break;

        case 0xE318261B: // CH- button pressed
        //stuff here open crossing gates
        {
       Serial.println("open crossing gates");
       //angle = 10;
       for(angle = 10; angle < 170; angle +=1)
       { 
       myservo.write(angle);
       delay(50);
       }
        }
                
        break;

        case 0xEE886D7F: // CH+ button pressed
        //stuff here to close crossing gates
        {
       Serial.println("close crossing gates");
       //angle = 170;
       for(angle = 170; angle > 10; angle -=1)
       {
       myservo.write(angle);
       delay(50);
       } 
        }
                
        break;
               
      }
    irrecv.resume(); // receive the next value
  }
}


// DIR ANTICLOCK = 1 , CLOCKWISE = -1)
// delaying PAUSE millisecs between steps
void moveMotor (int nsteps, int dir, int pause)
{
  int i,s;
  int t3 = nsteps-t2;
  int t4 = nsteps-t1;
  #ifdef DEBUG
    Serial.write("Turntable position was ");
    Serial.print(TTpos);
  #endif
  digitalWrite(LED1,HIGH);  // Red LED on for moving
  PORTD=(PORTD & 0x87) | motorStep[stepPos];  // turn stepper motor drive back on.
  delay(pause);            // wait a while
  for(i=0;i<nsteps;i++)
  {
    s = pause;
    if((i<t1)||(i>t4))
      s = s1;           // slowest speed
    if((i<t2)||(i>t3))
      s = s2;           // middle speed
    stepMotor(dir);     // step
    delay(s);           // and wait
  }
  PORTD=(PORTD & 0x87); // turn stepper motor drive off

  TTpos = TTpos + dir * nsteps; // update position
 if(TTpos<=-10250)       // we have rotated through -360 need to reset TTpos
    TTpos=TTpos+10250;
 if(TTpos>=10250)  // we have rotated through +360 need to reset TTpos
     TTpos=TTpos-10250;  // TTpos must now be in the range -10250 to +10250
     #ifdef DEBUG
    Serial.write(" Turntable position is ");
    Serial.print(TTpos);
    Serial.write("\n");
  #endif
  digitalWrite(LED1, LOW);  // Red LED off for stopped
}// end of moveMotor()

// move while hall effect sensor is in a particular state
void moveWhileHall (int dir, int pause, int state)
{
  int s;
  s=digitalRead(hallPin);
  while(s==state)
  {
    stepMotor(dir);
    delay(pause);
    s=digitalRead(hallPin);
  }
}// end of moveWhileHall()

// steps motor 1 step. (ANTICLOCK = 1 , CLOCKWISE = -1)
void stepMotor (int step)
{
  stepPos=stepPos+step;
  if(stepPos>=4)
    stepPos=0;
  else
  if(stepPos<0)
    stepPos=3;
    // hopefully leave other bits of PORTD as they were.
  PORTD=(PORTD & 0x87) | motorStep[stepPos];
}// end of stepMotor()
