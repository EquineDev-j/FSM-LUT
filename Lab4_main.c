//*****************************************************************************
//
// Lab 4 - "Finite State Machine"
// 1/6/22
// Jannel Bennett
//
// For this lab; Use a FSM to implement the control for a dehumidifier. Use LUT method of implementing a FSM.
//              A compressor cools a coil and a fan blows air from the space across the coil. Moisture in the air condenses onto the coil
//                  and runs down the drain.
//
// ** PIN 2.7 Not Working
//
// Initialize GPIO. Setup inputs and outputs as follows:
//
//  Port pin    in/out  Pullup/down     Connect
//    P1.1        In      Up               S1
//    P1.4        In      Up               S2
//    P2.0        Out     N/A             LED-R
//    P2.2        Out     N/A             LED-B
//    P2.5        In      N/A             LED - TA0.2
//
//  *********   Nokia LCD interface reference   **************
//
// Red SparkFun Nokia 5110 (LCD-10168)
// -----------------------------------
// Signal        (Nokia 5110) LaunchPad pin
// 3.3V          (VCC, pin 1) power
// Ground        (GND, pin 2) ground
// UCA3STE       (SCE, pin 3) connected to P9.4
// Reset         (RST, pin 4) connected to P9.3
// Data/Command  (D/C, pin 5) connected to P9.2
// UCA3SIMO      (DN,  pin 6) connected to P9.7
// UCA3CLK       (SCLK, pin 7) connected to P9.5
// back light    (LED, pin 8) not connected, consists of 4 3.3 V white LEDs which draw ~80mA total

//****************************************************************************

#include <stdint.h>
#include "msp.h"
#include "msoe_lib_clk.h"
#include "msoe_lib_delay.h"
#include "msoe_lib_lcd.h" // files for lcd commands/functions
#include "fsm.h"

// Function Prototypes //
void init_gpio(void);
void P1_int_setup(void);
void init_A2D(void);
void stupLCD(void);

#define INT_ADC14_BIT (1<<24)

// Global Variables //
float adc_Ambient;
int adc_RH;
int setpoint=60;
int b1=1;

/**
 * main.c
 */

void main(void){

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    Clock_Init_48MHz();   // run system at 48 MHz (default is 3 MHz)

    event input;
    state current = S1; // system on initially
    int RH;
    float Ambient;


    init_gpio();
    P1_int_setup();
    init_A2D();
    stupLCD();

    while(1){

        // read inputs - store result in input variable


        LCD_goto_xy(5, 0);  // start at row 3, col 8
        LCD_print_udec5(setpoint);

        LCD_goto_xy(4, 1);  // start at row 3, col 8
        Ambient =(50*(adc_Ambient/1024))+40;
        LCD_print_udec5(Ambient);

        LCD_goto_xy(3, 2);  // start at row 2, col 3
        RH = 100-(adc_RH*100/1023);
        LCD_print_udec5(RH);

        // ICE CHECK
        if ((P6->IN & BIT1)==0){ // switch up, ice
            LCD_goto_xy(8, 3);  // start at row 3, col 8
            LCD_print_str("ON ");

            input = Ice;
        }
        else { // switch down, no ice

            LCD_goto_xy(8, 3);  // start at row 3, col 8
            LCD_print_str("OFF");

            if (RH >= (setpoint + 5)){ // State ON
                input = sp_H;

            }
            else if (RH <= (setpoint - 5)){ // State OFF
                input = sp_L;
            }
            else {input = input;} // stay in current state
        }
        // call State machine function handler. Takes in type state - current and type event - input. Returns next state, type state.
       current = stateUpdate(current,input); // first time around current is S1, changes depending on conditions in LUT

       // add delay if needed
       // make sure delay isn't too long
       // otherwise will miss reading
       // if the button was pressed
       Delay_48MHz_ms(200);
    }
}//END

//********************FUNCTIONS**********************************************//
// initialize ports
void init_gpio(void)
{
      // set unused pins to pullup/down enabled to avoid floating inputs
   /* P1->REN |= 0xFF;
    P2->REN |= 0xFF;
    P3->REN |= 0xFF;
    P4->REN |= 0xFF;
    P5->REN |= 0xFF;
    P6->REN |= 0xFF;
    P7->REN |= 0xFF;
    P8->REN |= 0xFF;
    P9->REN |= 0xFF;
    P10->REN |= 0xFF;*/


    //R&B LED's init
    P2->DIR |= (BIT0 | BIT2);  // output
    P2->OUT &=~ (BIT0 | BIT2);  //start off

    //Push1 & Push2 init
    P1->DIR&=~(BIT1|BIT4); // set to zero for input
    P1->REN|=(BIT1|BIT4); // enable pullup
    P1->OUT |= (BIT1|BIT4);  // activate pullup


    // External Switch
    P6->DIR&=~(BIT1); // set to zero for input
    P6->REN|=(BIT1); // enable pullup
    P6->OUT|=(BIT1); // activate pullup

    // A6 (P4.7), A8 (P4.5) ADC input setup - relative Humidity / ambient Temp
    P4->SEL1 |= (BIT5 | BIT7);
    P4->SEL0 |= (BIT5 | BIT7);

      return;
}

// port 1 interrupt setup
void P1_int_setup(void) // button interrupt
{
    P1->IE |= (BIT1 | BIT4);  // enable interrupt on P1.1, P1.4
    P1->IES |= (BIT1 | BIT4); // falling edge
    NVIC->ISER[1] |= (1<<3);  // enable P1 interrupt
}

// initialize A/D
void init_A2D(void) // check settings
{
    // Sampling time, S&H=96, ADC14 on, SMCLK, repeat seq of channels, repeated conv.
    ADC14->CTL0 |= ADC14_CTL0_SHT0_5 | ADC14_CTL0_SHP | ADC14_CTL0_SSEL_4 | ADC14_CTL0_ON
             | ADC14_CTL0_CONSEQ_3  | ADC14_CTL0_MSC;
    ADC14->CTL1 |= ADC14_CTL1_RES_1;  // 10-bit conversion
    ADC14->CTL1 &= ~ADC14_CTL1_RES_2;  // 10-bit conversion
    ADC14->CTL1 |= (3 << ADC14_CTL1_CSTARTADD_OFS);  // start w/ MEM[3]
    ADC14->MCTL[3] |= ADC14_MCTLN_INCH_8; // input on A8 to mem3
    ADC14->MCTL[4] |= ADC14_MCTLN_INCH_6;  // input on A6 to mem4
    ADC14->IER0 |= ADC14_IER0_IE3 | ADC14_IER0_IE4;  // enable interrupt for MEM0 and MEM1
    ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;  // enable and start first

    NVIC->ISER[0] |= INT_ADC14_BIT;  // enable ADC interrupt in NVIC

}

// setup LCD
void stupLCD(void){
    LCD_Config();
    LCD_clear();

    LCD_goto_xy(0,0);
    LCD_print_str("SetP");

    LCD_goto_xy(0,1);
    LCD_print_str("Amb");

    LCD_goto_xy(0,2);
    LCD_print_str("RH");

    LCD_goto_xy(0,3);
    LCD_print_str("Defrost");
}

void PORT1_IRQHandler(void) // button interrupt duties
{
    uint16_t dummy; // dummy variable

        dummy = P1->IV;  // clear flag 'Interrupt Vector Reg' , store val dummy

        if((P1->IN&BIT1) == 0)setpoint+=5;
        else setpoint-=5;

}

void ADC14_IRQHandler(void) // adc update
  {

      // reading clears flag
      adc_Ambient = ADC14->MEM[3]; // store val from mem0 to adc_Ambient
      ADC14->CLRIFGR0|=BIT3; // clear pending interrupt flag


      adc_RH = ADC14->MEM[4]; // store val from mem1 to adc_RH
      ADC14->CLRIFGR0|=BIT4; // clear pending interrupt flag
  }
