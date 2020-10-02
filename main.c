//***************************************************************************************
//  SHARP LS013B7DH03 Memory Display
//
//  Simple library for TI MSP430FR5994 LaunchPad to write characters to SHARP LS013B7DH03
//  Memory Display using hardware SPI.
//
//  This code should work with any display of that series with compatible pinouts.
//
//  ACLK = n/a, MCLK = SMCLK = default DCO. Note that display specifies 1MHz max for SCLK
//
//                MSP430FR5994
//             -----------------
//            |             P1.0|-->LED  (VCOM status display)
//            |                 |
//            |             P1.1|-->LED  (DIAG passed Init Calls)
//            |                 |
//            |             P1.2|-->LCD Power
//            |                 |
//            |             P6.2|-->LCD_DISP (Toggle Display of LCD)
//            |                 |
//            |             P1.3|-->LCD_CS (SPI Chip Select for LCD Module)
//            |                 |
//            |             P5.2|-->SPI_CLK
//            |                 |
//            |             P5.0|-->SPI_CODI (Controller Out, Device In - Uses UCB1)
//            |                 |
//
//  Display VDD and VDDA connected to LaunchPad VCC
//  Display GND connected to LaunchPad GND
//
//  This code works with the TI BOOKSXL-SHARP128 booster pack
//
//  Zac Adam-MacEwen (Kensho Security Labs)
//  June 2020
//***************************************************************************************

#include "driverlib.h"
#include <msp430.h>
#include "font.h"

#define MLCD_WR 0x01                    // MLCD write line command
#define MLCD_CM 0x04                    // MLCD clear memory command
#define MLCD_SM 0x00                    // MLCD static mode command
#define MLCD_VCOM 0x02                  // MLCD VCOM bit

#define PIXELS_X 124                    // display is 124x124
#define PIXELS_Y 124                    // display is 124x124

volatile unsigned char VCOM;             // State of VCOM (either 0x00 or 0x02)

unsigned char bufferLine[PIXELS_X/8];   // Textmode Line Buffer
char bufferText[9];

volatile unsigned int timeMSec;         // clock milliseconds
volatile unsigned char timeSecond;      // clock seconds
volatile unsigned char timeMinute;      // clock minutes

void Init_SPI(void) {
    EUSCI_B_SPI_disable(EUSCI_B1_BASE); // disable the EUSCI to be programme

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5,GPIO_PIN0,GPIO_PRIMARY_MODULE_FUNCTION);
         // Select Port 5, Set Pin 0 to output Primary Module Function, (UCB1TXD/UCB1SIMO)

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5,GPIO_PIN2,GPIO_PRIMARY_MODULE_FUNCTION);
         // Set port 5, Pin 2 to output Primary Module Function, UCB1CLK.

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,GPIO_PIN1,GPIO_PRIMARY_MODULE_FUNCTION);
         // Select Port 5, Set Pin 1 to input Primary Module Function, ( UCB1RXD/UCB1SOMI)
         // We don't need this pin, technically, but it's included for completeness.

         // Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    PMM_unlockLPM5();

         //Initialize UCB1 as SPI Controller

    EUSCI_B_SPI_initMasterParam param = {0};                           // param is struct data type

    param.selectClockSource = EUSCI_B_SPI_CLOCKSOURCE_SMCLK;
    param.clockSourceFrequency = 1000000;   // System might run higher, may need to adjust.
    param.desiredSpiClock = 500000;  // Theoretical maximum for the target LCD module.
    param.msbFirst = EUSCI_B_SPI_LSB_FIRST; // LCD module is little-endian.
    param.clockPhase = EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT; //not certain correct
    param.clockPolarity = EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW;  // not certain correct
    param.spiMode = EUSCI_B_SPI_3PIN;

    EUSCI_B_SPI_initMaster(EUSCI_B1_BASE, &param);     //   ***  B SPI is initialized with struct param data

         //Wait for lines to stabilize prior to switching on the SPI
    __delay_cycles(100);
         //Enable SPI module
    EUSCI_B_SPI_enable(EUSCI_B1_BASE);   // clear the UCSWRST bit to enable the SPI

    EUSCI_B_SPI_clearInterrupt(EUSCI_B1_BASE,EUSCI_B_SPI_RECEIVE_INTERRUPT);

    // Disable USCI_B1 RX interrupt - we expect nothing back
    EUSCI_B_SPI_disableInterrupt(EUSCI_B1_BASE,EUSCI_B_SPI_RECEIVE_INTERRUPT);

    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN3);     // Set P1.3 as output pin for CS of LCD
    GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN3);  // disable LCD Module
         //Wait for devices to initialize.
    __delay_cycles(100);

}

void Init_LCD(void) {

    // We need a function to initialize the LCD, which is non-automatic.
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN2);// Provide power to the LCD's controller.
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN2); // Provide display power to the LCD.
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN2);
    GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN3);
    EUSCI_B_SPI_transmitData(EUSCI_B1_BASE, (MLCD_CM | VCOM));// Send Clear Command with appropriate VCOM bit state (VOM initialized elsewhere)
    EUSCI_B_SPI_transmitData(EUSCI_B1_BASE, 0x00);// clear command needs an enter button
    GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN3);
}

void Init_GPIO(void) {
    //We have a couple GPIO pins that most make sense to set up individually.
    // Some are explicitly or implicitly set up during Init_SPI.
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0); // P1.0 is an LED for indicating VCOM state.
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0); // Initial VCOM = 0
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN1); // P1.1 is a debugging LED to make sure we hit the int.
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN1);

    //Uncomment the section below for the debugging panel.
    //GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0); //Current use:
    //GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0);
    //GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN1); //Current use:
    //GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN1);
    //GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN2); //Current use:
    //GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN2);
    //GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN3); //Current use:
    //GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN3);
    //GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN4); //Current use:
    //GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN4);
    //GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN5); //Current use:
    //GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);
    //GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6); //Current use:
    //GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);
    //GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN7); //Current use:
    //GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);
}

void Init_Timers(void) {
    // There are some very important global timers that we need in order to control the time
    // display as well as to toggle the VCOM bit.
    VCOM = MLCD_VCOM; // Starting value setup.
    timeMSec = 0;                                       // initialize variables used by "clock"
    timeSecond = 0;
    timeMinute = 0;
    // Set master clock to 1MHz; at this rate TimerA would interrupt every millisecond.
    // This is not especially power performant.
    // sets the properties to init Timer_A
    CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_0); // Set DCO frequency 1 MHz
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1); //SMCLK = 1 Mhz
    Timer_A_initUpModeParam initContParam = {0};
    initContParam.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    initContParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    initContParam.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    initContParam.timerPeriod = 1000; // this should result in a roughly 1ms period.
    initContParam.timerClear = TIMER_A_DO_CLEAR;
    initContParam.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE;
    initContParam.startTimer = false;
    Timer_A_initUpMode(TIMER_A0_BASE, &initContParam); // init TA in compare mode for TA2
    Timer_A_clearTimerInterrupt(TIMER_A0_BASE);
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0, 1000);
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    Timer_A_enableInterrupt(TIMER_A0_BASE);
    Timer_A_enableCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    // start the timer
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
}

// Reverse the bit order of a byte to resolve the mirroring issue. Likely used in all SPI sends.
// Full credit to Stack Overflow user STH for this handy function
unsigned char reverse(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

//Transfer the line buffer to the display via SPI
// Input is the line number where the buffer should be rendered.
void SPIWriteLine(unsigned char line)
{
    GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN3); // Take SCS low to talk to the display
    EUSCI_B_SPI_transmitData(EUSCI_B1_BASE, (MLCD_WR | VCOM));// send the command to write lines.
    EUSCI_B_SPI_transmitData(EUSCI_B1_BASE, line); // send the line address to the system

    unsigned char index_of_byte = 0; //loop to send line buffer byte by byte
    while (index_of_byte <= (PIXELS_X/8))
        {
            EUSCI_B_SPI_transmitData(EUSCI_B1_BASE, ~reverse(bufferLine[index_of_byte++]));
            while (UCB1STATW & UCBUSY);
        }

    // send 16 bits to latch buffers and end transfer
    EUSCI_B_SPI_transmitData(EUSCI_B1_BASE, 0x00);
    EUSCI_B_SPI_transmitData(EUSCI_B1_BASE, 0x00);
    GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN3); // taks eSCS high
}

// write a string to display, truncated after PIXEL_X/8 characters
// input: text      0-terminated string
//        line      vertical position of text, 0-(PIXEL_Y-1)
void printText(const char* text, unsigned char line)
{
    unsigned char character, bitmap, indexText, indexLineBuffer, indexLine;
    bool padded;
    //For simplicity, and because we are borrowing an existing font library, we print line-by-line
    indexLine = 0;
    while(indexLine < 8 && line < PIXELS_Y)             //loop for 8 char lines within display
    {
        indexText = 0;
        indexLineBuffer = 0; //TODO: Resume from here, we never seem to leave the loop starting line 187
        while(indexLineBuffer < (PIXELS_X/8) && (character = text[indexText]) != 0)  // We did not reach the end of the line or the string.
        {
            if(character < ' ' || character > 'Z')  //Sanity: replace characters not in font.
            {
                character = ' ';
            }
            character = character - 32; //Changes character to an index in our stolen font table.
            bitmap = font8x8[(character*8)+indexLine];  // Retrieves the byte defining one line of character.

            bufferLine[indexLineBuffer] = bitmap;
            indexLineBuffer++;
            indexText++;
        }

        while(indexLineBuffer < (PIXELS_X/8))  //Pad line for empty characters.
        {
            padded = true;
            bufferLine[indexLineBuffer] = 0x00;
            indexLineBuffer++;
        }

        if(padded) // needed because <= might break things.
        {
            bufferLine[indexLineBuffer] = 0x00;
            padded = false;
        }

        SPIWriteLine(line++);
        indexLine++;
    }
}

int main(void) {

    WDTCTL = WDTPW | WDTHOLD; // Disable the watchdog timer. We might rely on this later, but not for now.
    P1IFG = 0;
    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    SFRIFG1 &= ~OFIFG; // Clear the OFIFG because that's some annoying bugs.
    PMM_unlockLPM5();
    Init_GPIO();
    Init_Timers();
    Init_SPI();
    Init_LCD();
    printText("HELLO, WORLD!", 1);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN1);
    printText("MSP430FR5994", 16);
    printText("SAYS", 24);
    printText("HI", 32);
    printText("1234567890123456", 56);

    while (1)
    {
        //Show the current VCOM state.
        if (VCOM == 0x00)
        {
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
        }
        else
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
        }
        // write clock to display by forming a string literal representing the current time
        bufferText[0] = ' ';
        bufferText[1] = timeMinute / 10 + '0';
        bufferText[2] = timeMinute % 10 + '0';
        bufferText[3] = ':';
        bufferText[4] = timeSecond / 10 + '0';
        bufferText[5] = timeSecond % 10 + '0';
        bufferText[6] = 0;
      printText("UPTIME:",64);
      printText(bufferText,72);
      GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN3);
      EUSCI_B_SPI_transmitData(EUSCI_B1_BASE, (MLCD_SM | VCOM));
      EUSCI_B_SPI_transmitData(EUSCI_B1_BASE, 0);
      GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN3);
        __bis_SR_register(LPM0_bits | GIE);                       // enable interrupts and go to sleep
        //
    };
    return (0);
}



// interrupt service routine to handle timer A
#pragma vector=TIMER0_A0_VECTOR
__interrupt void VCOM_ISR (void)
{
    timeMSec++;// count milliseconds
    PMM_unlockLPM5();
    Timer_A_clearTimerInterrupt(TIMER_A0_BASE);
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    if(timeMSec == 1000)                                // if we reached 1 second
    {
        timeMSec = 0;                                   // reset milliseconds
        timeSecond++;                                   // increase seconds
        if(timeSecond == 60)                            // if we reached 1 minute
        {
            timeSecond = 0;                             // reset seconds
            timeMinute++;                               // increase minutes
            if(timeMinute == 60)                        // if we reached 1 hour
            {
                timeMinute = 0;                         // reset minutes
            }
        }

        if(VCOM == 0x00)                                  // invert polarity bit every second
        {
            VCOM = MLCD_VCOM;
        }
        else
        {
            VCOM = 0x00;
        }
        __bic_SR_register_on_exit(LPM0_bits);            // wake up main loop every second
    }
}
