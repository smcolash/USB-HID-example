/**
 *  Example HID device firmware for a PIC16F1455. Specifically developed for
 *  the USB PIC16F1455 Breakout Board Kit from ubld.it
 *  (http://ubld.it/products/usb-pic16f1455-breakout-board/).
 *
 *  Based on the M-Stack software by Alan Ott, Signal 11 Software.
 *
 *  Copyright (C) 2017 Scott McOlash
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a 
 *  copy of this software and associated documentation files (the "Software"), 
 *  to deal in the Software without restriction, including without limitation 
 *  the rights to use, copy, modify, merge, publish, distribute, sublicense, 
 *  and/or sell copies of the Software, and to permit persons to whom the 
 *  Software is furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in 
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 *  DEALINGS IN THE SOFTWARE.
 */

#include "usb.h"
#include <xc.h>
#include <string.h>
#include "usb_config.h"
#include "usb_ch9.h"
#include "usb_hid.h"

/**
 * Set the operating frequency of the internal clock.
 */
#define _XTAL_FREQ 48000000

/**
 * Initialize the PIC16F1455 internal clock.
 * See http://ww1.microchip.com/downloads/en/DeviceDoc/40001639B.pdf.
 */
void oscillator_init (void)
  {
    //
    // enable the 48MHz internal clock
    //
    OSCCONbits.IRCF = 0b1111;

    //
    // enable clock tuning from USB
    //
    ACTCONbits.ACTSRC = 1;
    ACTCONbits.ACTEN = 1;

    return;
  }

/**
 * Initialize the PIC16F1455 timer and associated interrupt.
 * See http://ww1.microchip.com/downloads/en/DeviceDoc/40001639B.pdf.
 */
void timer_init (void)
  {
    return; // for now, don't enable the timer and its interrupt...
    
    //Timer2 Registers Prescaler= 4 - TMR2 PostScaler = 12 - PR2 = 250 - Freq = 1000.00 Hz - Period = 0.001000 seconds
    T2CON |= 88; // bits 6-3 Post scaler 1:1 thru 1:16
    T2CONbits.T2CKPS1 = 0; // bits 1-0  Prescaler Rate Select bits
    T2CONbits.T2CKPS0 = 1;
    PR2 = 250; // PR2 (Timer2 Match value)

    INTCONbits.GIE = 1; // global interrupts enabled
    INTCONbits.PEIE = 1; // peripheral interrupts enabled
    PIE1bits.TMR2IE = 1; // timer 2 interrupt enabled

    T2CONbits.TMR2ON = 1; // turn timer 2 on

    return;
  }

/**
 * Initialize the PIC16F1455 GPIO pins for simple input and output.
 * See http://ww1.microchip.com/downloads/en/DeviceDoc/40001639B.pdf.
 */
void gpio_init (void)
  {
    //
    // disable the external adc input pins
    //
    ANSELA = 0;
    ANSELC = 0;

    //
    // configure C0 as output
    //
    TRISCbits.TRISC0 = 0;

    //
    // configure C1 as output
    //
    TRISCbits.TRISC1 = 0;

    //
    // configure C2 as output
    //
    TRISCbits.TRISC2 = 0;

    //
    // configure C3 as output
    //
    TRISCbits.TRISC3 = 0;

    //
    // configure C4 as input
    //
    TRISCbits.TRISC4 = 1;

    //
    // configure C5 as output for PWM1
    //
    TRISCbits.TRISC5 = 1;


// Configure PWM for free running mode
//
// PWM period = Tcy * prescale * PTPER = 0.33ns * 64 * PTPER
// PWM pulse width = (Tcy/2) * prescale * PDCx
//
//-- PWM1CON = 0xF0; // Enable all PWM pairs in complementary mode

//PTCONbits.PTCKPS = 3; // prescale=1:64 (0=1:1, 1=1:4, 2=1:16, 3=1:64)
//PTPER = 14063; // 30ms PWM period (15-bit period value)
//PDC1 = 1875; // 2ms pulse width on PWM channel 1
//PTMR = 0; // Clear 15-bit PWM timer counter
//PTCONbits.PTEN = 1; // Enable PWM time base

//PDC1 = 1875;
//PDC1 = 100;

    //
    // turn off the outputs
    //
    LATCbits.LATC0 = 0;
    LATCbits.LATC1 = 0;
    LATCbits.LATC2 = 0;
    LATCbits.LATC3 = 0;

    return;
  }

/**
 * Initialize the PIC16F1455 internal thermal sensing.
 * See http://ww1.microchip.com/downloads/en/DeviceDoc/40001639B.pdf and
 * http://ww1.microchip.com/downloads/en/AppNotes/01333A.pdf.
 */
void thermometer_init (void)
  {
    FVRCONbits.TSEN = 1; // enable thermometer
    FVRCONbits.TSRNG = 1; // high range: Vout = Vdd - 4*Vt

    ADCON0bits.CHS = 0b11101; // select temp sensor channel

    ADCON1bits.ADFM = 1; // adc output right justified
    ADCON1bits.ADCS = 0b111; // use internal Frc oscillator
    ADCON1bits.ADPREF = 0b00; // Vref internally connected to Vdd

    ADCON0bits.ADON = 1; // enable ADC
    __delay_ms(1); // give adc time to charge

    return;
  }

/**
 * The main device function.
 */
int main (void)
  {
    uint8_t *TX;
    const uint8_t *RX;

    //
    // configure the basic hardware
    //
    hardware_init ();

    //
    // configure the timer
    //
    oscillator_init ();

    //
    // configure the input/output pins
    //
    gpio_init ();

    //
    // configure the timer interrupt
    //
    timer_init ();

    //
    // configure the thermal sensing
    //
    thermometer_init ();

    //
    // configure the USB components
    //
    usb_init ();

    //
    // get a pointer to the USB transmit (to host) data buffer
    //
    TX = usb_get_in_buffer (1);

    //
    // loop forever
    //
    while (true)
      {
        //
        // check for received data or for a condition to process
        //
#ifndef USB_USE_INTERRUPTS
        usb_service ();
#endif

        //
        // skip further actions is the USB stack is not yet configured
        //
        if (!usb_is_configured ())
          {
            continue;
          }

        //
        // skip further actions if the USB stack is not ready
        //
        if (usb_in_endpoint_halted (1) || usb_in_endpoint_busy (1))
          {
            continue;
          }

        //
        // skip further actions if the USB stack has not received any data
        //
        if (!usb_out_endpoint_has_data (1))
          {
            continue;
          }

        //
        // get a pointer to the received USB data
        //
        usb_get_out_buffer (1, &RX);

        //
        // clear the transmit buffer contents
        //
        memset ((char *) TX, 0x00, EP_1_IN_LEN);

        //
        // send the request code as the first byte of the response
        //
        TX[0] = RX[0];

        //
        // handle the request code
        //
        switch (RX[0])
          {
            //
            // get the device status, etc.
            //
            case 0x00:
              TX[1] = 0x01;
              TX[2] = 0x02;
              TX[3] = 0x03;
              TX[4] = 0x04;
              TX[5] = 0x05;
              TX[6] = 0x06;
              TX[7] = 0x07;
              usb_send_in_buffer (1, EP_1_IN_LEN);
              break;

            //
            // set a GPIO output pin state
            //
            case 0x01:
              TX[1] = RX[1];
              TX[2] = 0xff;
              TX[3] = 0xff;

              switch (RX[1])
                {
                  //
                  // handle RC0
                  //
                  case 0:
                    TX[2] = !(PORTCbits.RC0 == 0);
                    LATCbits.LATC0 = !(RX[2] == 0);
                    TX[3] = !(PORTCbits.RC0 == 0);
                    break;

                  //
                  // handle RC1
                  //
                  case 1:
                    TX[2] = !(PORTCbits.RC1 == 0);
                    LATCbits.LATC1 = !(RX[2] == 0);
                    TX[3] = !(PORTCbits.RC1 == 0);
                    break;

                  //
                  // handle RC2
                  //
                  case 2:
                    TX[2] = !(PORTCbits.RC2 == 0);
                    LATCbits.LATC2 = !(RX[2] == 0);
                    TX[3] = !(PORTCbits.RC2 == 0);
                    break;

                  //
                  // handle RC3
                  //
                  case 3:
                    TX[2] = !(PORTCbits.RC3 == 0);
                    LATCbits.LATC3 = !(RX[2] == 0);
                    TX[3] = !(PORTCbits.RC3 == 0);
                    break;
                }

              usb_send_in_buffer (1, EP_1_IN_LEN);
              break;

            //
            // get a GPIO output pin state
            //
            case 0x02:
              TX[1] = RX[1];
              TX[2] = 0xff;

              switch (RX[1])
                {
                  //
                  // handle RC0
                  //
                  case 0:
                    TX[2] = !(PORTCbits.RC0 == 0);
                    break;

                  //
                  // handle RC1
                  //
                  case 1:
                    TX[2] = !(PORTCbits.RC1 == 0);
                    break;

                  //
                  // handle RC2
                  //
                  case 2:
                    TX[2] = !(PORTCbits.RC2 == 0);
                    break;

                  //
                  // handle RC3
                  //
                  case 3:
                    TX[2] = !(PORTCbits.RC3 == 0);
                    break;

                  //
                  // handle RC4
                  //
                  case 4:
                    TX[2] = !(PORTCbits.RC4 == 0);
                    break;

                  //
                  // handle RC5
                  //
                  case 5:
                    TX[2] = !(PORTCbits.RC5 == 0);
                    break;
                }

              usb_send_in_buffer (1, EP_1_IN_LEN);
              break;

            //
            // get the CPU temperature in degrees Fahrenheit per AN1333
            //
            case 0x03:
              ADCON0bits.ADGO = 1; // start ADC measurement
              while (ADCON0bits.ADGO != 0); // wait for conversion
              unsigned int tempADC = (ADRESH << 8) + ADRESL;
              float temperature = 0.925679 * tempADC - 487.727;
              TX[1] = (int) temperature;

              // debug...
              TX[1] = ADRESL;
              TX[2] = ADRESH;

              usb_send_in_buffer (1, EP_1_IN_LEN);
              break;

            //
            // request a reset of the device (TODO)
            //
            case 0x7f:
              TX[1] = 1;
              TX[2] = 1;
              TX[3] = 1;
              TX[4] = 1;
              TX[5] = 1;
              TX[6] = 1;
              TX[7] = 1;
              usb_send_in_buffer (1, EP_1_IN_LEN);
              //RESET ();
              break;

            //
            // all other request codes are invalid
            //
            default:
              TX[1] = 0xff;
              usb_send_in_buffer (1, EP_1_IN_LEN);
              break;
          }

        //
        // re-arm USB endpoint 1 to receive more data
        //
        usb_arm_out_endpoint (1);
      }

    //
    // this instruction should _never_ be reached
    //
    return (1);
  }

/**
 * TBD...
 */
int8_t app_unknown_setup_request_callback (const struct setup_packet *setup)
  {
    return (process_hid_setup_request (setup));
  }

/**
 * Interrupt callback specialized to handle the timer and the USB interrupts.
 */
void interrupt isr (void)
  {
    //
    // handle the timer interrupt
    //
    if (PIR1bits.TMR2IF)
      {
        //
        // reset the timer interrupt flag
        //
        PIR1bits.TMR2IF = 0;
      }

#ifdef USB_USE_INTERRUPTS
    //
    // handle the USB interrupt
    //
    if (1) // FIXME - should check for the USB interrupt...
      {
        usb_service ();
      }
#endif

    return;
  }
