#include <stdio.h>
#include <driverlib.h>
#include <string.h>
#include "stdlib.h"
#include <msp430.h>

#define I2C_DEV_ADDR 0x02
#define N2 1000
uint8_t samples[N2];
typedef enum {
    receive_trigger,
    transmit_pulse,
    receive_pulse,
    send_data
} states_t;
volatile states_t state = receive_trigger;

void lfsr_galois(uint32_t *out, uint32_t start_state)
{
    uint32_t lfsr = start_state;

    unsigned msb = (int32_t) lfsr < 0;   // Get MSB (i.e., the output bit).
    lfsr <<= 1;                          // Shift register
    if (msb)                             // If the output bit is 1,
        lfsr ^= 0x000000C5u;             // apply toggle mask.
    out[0]= lfsr;
    out[1]=(int32_t) lfsr < 0;
}

void nbit_lfsr(uint32_t *signature, uint32_t n, uint32_t seed)
{
    uint32_t out[2]={0};
    uint32_t start_state=seed;
    int i;
    for(i=0;i<n;i++)
    {
        lfsr_galois(out, start_state);
        start_state=out[0];
        signature[i]=out[1];
    }

}

uint32_t lfsr(uint32_t n, uint32_t seed)
{
    uint32_t signature_array[32]={0};
    uint32_t signature=0;
    int j;

    nbit_lfsr(signature_array, n, seed);
    for(j=0;j<n;j++)
    {
        signature|=signature_array[j]<<(n-j-1);
    }

    return signature;
}

void SetTrigger(uint16_t code, unsigned int codeLength)
{
    unsigned int i;
    for(i=1; i <= codeLength; i++)
    {
        unsigned int code_new=((code>>(codeLength-i))&0x0001);//&0x0001
        if(code_new==0)
        {
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);     // Flash Test1 LED with pseudo-random number

            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);     // set PZT_POUT1 (P2.5) to generate same number
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);     // set PZT_POUT2 (P4.1) to generate same number

            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN6);    // set PZT_NOUT1 (P2.6) to 180 degree phase offset
            GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2);    // set PZT_NOUT2 (P4.2) to 180 degree phase offset

        }
        else
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);    // Flash Test1 LED with pseudo-random number

            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);    // set PZT_POUT1 (P2.5) to generate same number
            GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1);    // set PZT_POUT2 (P4.1) to generate same number

            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6);     // set PZT_NOUT1 (P2.6) to 180 degree phase offset
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);     // set PZT_NOUT2 (P4.2) to 180 degree phase offset

        }
 //       __delay_cycles(500); //pulse width 5 us
    }
}

void tx_init(void)
{
    // Set P1.0 to output direction
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);                   // TEST1

    // Set P2.4, P2.5, P2.6, P4.0, P4.1, P4.2 to output direction
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN4);                   // RX_MODE1
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN5);                   // PZT_POUT1
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN6);                   // PZT_NOUT1
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0);                   // RX_MODE2
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN1);                   // PZT_POUT2
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN2);                   // PZT_NOUT2

    // Set P1.1 and P1.4 to output direction
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN1);                   // RX_EN1
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN4);                   // RX_EN2

    // Set P3.0 and P3.1 to input direction
    GPIO_setAsInputPin(GPIO_PORT_P3, GPIO_PIN0);                    // RX_OUT1
    GPIO_setAsInputPin(GPIO_PORT_P3, GPIO_PIN1);                    // RX_OUT2

    // Write low voltage to RX_MODE1 (P2.4) and RX_MODE2 (P4.0)
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);
}

void I2Cslave_init(void)
{
    // Configure USCI_B0 for I2C mode
    P1SEL0    |= BIT2 + BIT3;                     // Set I2C pins
    UCB0CTLW0  = UCSWRST;                         // Enable SW reset
    UCB0CTLW0 |= UCMODE_3 + UCSYNC;               // I2C Slave Mode
    UCB0I2COA0 = I2C_DEV_ADDR | UCOAEN;           // Enable Own Address 0x02
    UCB0CTLW0 &= ~UCSWRST;                        // Clear SW reset
    __bis_SR_register(GIE);                       // Enable global interrupts
//    UCB0IE     = UCSTTIE + UCRXIE0 + UCTXIE0;     // Enable STT, RX0 and TX0 interrupts
}

// create temporary data to transmit via I2C
void receivePulse(void) {
    unsigned int i;
    for(i = 0; i < N2; i++) {
        samples[i] = i;
    }
}

#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
    switch(__even_in_range(UCB0IV, 0x1e))
    {
        case USCI_NONE:          break;                     // no interrupt
        case USCI_I2C_UCALIFG:   break;                     // UCALIFG (Arbitration lost)
        case USCI_I2C_UCNACKIFG: break;                     // UCNACKIFG (NACK)
        case USCI_I2C_UCSTTIFG:  break;                     // UCSTTIFG (Start condition)
        case USCI_I2C_UCSTPIFG:  break;                     // UCSTPIFG (Stop condition)
        case USCI_I2C_UCRXIFG3:  break;                     // UCRXIFG3 (Slave 3 receive)
        case USCI_I2C_UCTXIFG3:  break;                     // UCTXIFG3 (Transmit buffer 3 empty)
        case USCI_I2C_UCRXIFG2:  break;                     // UCRXIFG2 (Slave 2 receive)
        case USCI_I2C_UCTXIFG2:  break;                     // UCTXIFG2 (Transmit buffer 2 empty)
        case USCI_I2C_UCRXIFG1:  break;                     // UCRXIFG1 (Slave 1 receive)
        case USCI_I2C_UCTXIFG1:  break;                     // UCTXIFG1 (Transmit buffer 1 empty)
        case USCI_I2C_UCRXIFG0:                             // UCRXIFG0 (Slave 0 receive)
            state = transmit_pulse;
//            if (state == receive_trigger) {
//                if (UCB0RXBUF == 0xFF) {
//                    state = transmit_pulse;
//                }
//                else {
//                    state = receive_trigger;
//                }
//            }
            break;
        case USCI_I2C_UCTXIFG0:                             // UCTXIFG0 (Transmit buffer empty)
            if (state == send_data) {
                unsigned int i;
                for (i = 0; i < N2; i++) {
                    UCB0TXBUF = samples[i];
                }
            }
            break;
        case USCI_I2C_UCBCNTIFG: break;                     // UCBCNTIFG: (CNT reached)
        case USCI_I2C_UCCLTOIFG: break;                     // UCCLTOIFG: (Clock Low Time Out)
        case USCI_I2C_UCBIT9IFG: break;                     // UCBIT9IFG: (9th bit)
        default: break;
    }
}

int main(void) {

    int const n = 8;
    uint32_t const seed = 0x72ad0009;
    uint32_t signature = 0;
    signature = lfsr(n, seed);                              // 10-bit pseudo random number output using a 32-bit LFSR

    WDTCTL = WDTPW | WDTHOLD;
    PMM_unlockLPM5();

    I2Cslave_init();                                        // Initialize I2C module
    tx_init();                                              // Initialize pins for transmitter

    while(1) {
        switch (state) {
            case receive_trigger:
                while(!(UCB0IFG & UCRXIFG0));               // wait until I2C receive
                if (UCB0RXBUF == 0xFF) {
                    state = transmit_pulse;
                }
                else {
                    state = receive_trigger;
                }
                UCB0IFG &= ~UCRXIFG0;
                break;
            case transmit_pulse:
                SetTrigger(signature, n);
                state = receive_pulse;
                break;
            case receive_pulse:
                receivePulse();
                state = send_data;
                break;
            case send_data:
                while(!(UCB0IFG & UCTXIFG0));
                unsigned int i;
                for (i = 0; i < N2; i++) {
                    UCB0TXBUF = samples[i];
                }
                UCB0IFG &= ~UCTXIFG0;
                state = receive_trigger;
                break;
        }
    }
}
