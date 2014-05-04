#include <hidef.h>      /* common defines and macros */
#include <mc9s12dg256.h>     /* derivative information */
#include "main.h"
#pragma LINK_INFO DERIVATIVE "mc9s12dg256b"
#include "main_asm.h" /* interface to the assembly module */

////////////////////////////////////////
// Initalize                          //
// Ports, Interrupts, Motor and Servo //
////////////////////////////////////////
void portInit(void){
  PLL_init(); // set system clock frequency to 24 MHz
  
  //Direction registers
  DDRH = 0x00;  // Port H is input       
  DDRB = 0x0F; // Port B starts as output
  PORTB = 0x00; // turn all LED's OFF
  DDRT = 0xFF;  // port T7 is input rest is output

  //motors
  PTT_PTT0 = 0; //default
  PTT_PTT1 = 0; //default
  PTT_PTT2 = 0; //default
  PTT_PTT3 = 0; //default
  motor0_init(); //left motor
  motor1_init(); //right motor
  
  //initialize a/d
  ad0_enable(); //enable a/d converter w/ interrupt with custom values
  RTI_init(); //turn on real time interrupt
}

void main(void) {
  portInit();  //initialize ports, interrupts, and motors
  for(;;); //do nothing forever (interrupt driven)
}


///////////
// Setup //
///////////
void setup() {
  if (PORTB_BIT4) {
    int i;
    PORTB_BIT0 = 0; 
    PORTB_BIT1 = 0; 
    PORTB_BIT2 = 0; 
    PORTB_BIT3 = 0;
    for(i = 0; i < 10; i++) { //flash LEDs for 250ms to show start
     PORTB_BIT0 = ~PORTB_BIT0;
     PORTB_BIT1 = ~PORTB_BIT1;
     PORTB_BIT2 = ~PORTB_BIT2;
     PORTB_BIT3 = ~PORTB_BIT3;
     ms_delay(25);
    }  
    PORTB_BIT0 = 0; 
    PORTB_BIT1 = 0; 
    PORTB_BIT2 = 0; 
    PORTB_BIT3 = 0;
    isSetup = 0x01; //Full power to pickup motor
    //PTT_PTT1 = 1; //Motor SLEW
    PTT_PTT0 = 1; //Motor EN
  } else {
    mid = s4avg-s5avg;
  }
}

//////////////////////////
// Forward Proportional //
// Wall follow          //
//////////////////////////
void forwardP() {
  //error = gWallCurrent - s5avg; //instant error e
  error = s4avg-s5avg;
  routput = (int)((BASE)-(Kp*error));
  loutput = (int)((BASE)+(Kp*error));
  
  // saturation logic
	if(routput > MAX)  // no motor values > 255 or < 0
		routput = MAX;
	  else if(routput <= MIN)
		  routput = MIN;
	if(loutput > MAX)  // no motor values > 255 or < 0
		loutput = MAX;
	  else if(loutput <= MIN)
		  loutput = MIN;

  PTT_PTT3 = 0; //Motor L1
  PTT_PTT5 = 1; //Motor L2
  PTT_PTT4 = 0; //Motor R1
  PTT_PTT6 = 1; //Motor R2
	  
	if (loutput == MIN) { 
    PTT_PTT3 = 1; //Motor L1
    PTT_PTT5 = 1; //Motor L2
	}  
	if (routput == MIN) { 
    PTT_PTT6 = 1; //Motor R1
    PTT_PTT7 = 1; //Motor R2
	} 
  
  //pwm
  motor0(routput); //Right motor
  motor1(loutput); //Left motor

}
  
////////////////////////////////////////
// Real time interrupt                //
// Higher priority than ATD interrupt //
////////////////////////////////////////
void interrupt 7 RTIhandler()
{  
  if (isSetup == 0x00) { 
    setup();
  } else {
    
    forwardP();
  }
  clear_RTI_flag();
}

////////////////////////////////////////////////
// A/D Converter interrupt                    //
// enabled through custom assembly subroutine //
////////////////////////////////////////////////
void interrupt 22 ANhandler()
{
    s4 = ATD0DR0L; // ATD converter 0 data register 0 lower 8 bytes
    s5 = ATD0DR1L; // ATD converter 0 data register 1 lower 8 bytes
    s6 = ATD0DR2L; // ATD converter 0 data register 2 lower 8 bytes
    s7 = ATD0DR3L; // ATD converter 0 data register 3 lower 8 bytes
    
    s4avg -= s4avg/sn;
    s5avg -= s5avg/sn;
    s6avg -= s6avg/sn;
    s7avg -= s7avg/sn;
    s4avg += s4/sn; // store front IR value in sensor array
    s5avg += s5/sn; // store side IR (LONG) value in sensor array
    s6avg += s6/sn; // store line IR in sensor array
    s7avg += s7/sn; // store side IR (SHORT) in sensor array
    sn++;
    if (sn > 255)
      sn = 10;
}

//////////////
// Movement //
//////////////
void stop() {
      motor0(0xFF);
      motor1(0xFF);
      PTT_PTT3 = 1; //Motor L1
      PTT_PTT5 = 1; //Motor L2
      PTT_PTT4 = 1; //Motor R1
      PTT_PTT6 = 1; //Motor R2
}

void forward(int speed) { 
      motor0(speed);
      motor1(speed);
      PTT_PTT3 = 0; //Motor L1
      PTT_PTT5 = 1; //Motor L2
      PTT_PTT4 = 0; //Motor R1
      PTT_PTT6 = 1; //Motor R2
}

void left(int rspeed){    
      motor0(rspeed);
      motor1(0xFF);
      PTT_PTT3 = 1; //Motor L1
      PTT_PTT5 = 1; //Motor L2
      PTT_PTT4 = 0; //Motor R1
      PTT_PTT6 = 1; //Motor R2
}

void leftManual(int rspeed, int lspeed){    
      motor0(rspeed);
      motor1(lspeed);
      PTT_PTT3 = 1; //Motor L1
      PTT_PTT5 = 0; //Motor L2
      PTT_PTT4 = 0; //Motor R1
      PTT_PTT6 = 1; //Motor R2
}

void back(int speed){
      motor0(speed);
      motor1(speed);
      PTT_PTT3 = 1; //Motor L1
      PTT_PTT5 = 0; //Motor L2
      PTT_PTT4 = 1; //Motor R1
      PTT_PTT6 = 0; //Motor R2
}

void backSplit(int rspeed, int lspeed){
      motor0(rspeed);
      motor1(lspeed);
      PTT_PTT3 = 1; //Motor L1
      PTT_PTT5 = 0; //Motor L2
      PTT_PTT4 = 1; //Motor R1
      PTT_PTT6 = 0; //Motor R2
}

