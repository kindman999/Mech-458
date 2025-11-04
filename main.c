// READ EVERY SINGLE WORD IN THIS PIECE OF CODE...IF YOU DON'T YOU WILL NOT UNDERSTAND THIS!!!!!!!
// READ EVERY SINGLE WORD IN THIS PIECE OF CODE...IF YOU DON'T YOU WILL NOT UNDERSTAND THIS!!!!!!!
// READ EVERY SINGLE WORD IN THIS PIECE OF CODE...IF YOU DON'T YOU WILL NOT UNDERSTAND THIS!!!!!!!
// READ EVERY SINGLE WORD IN THIS PIECE OF CODE...IF YOU DON'T YOU WILL NOT UNDERSTAND THIS!!!!!!!
// READ EVERY SINGLE WORD IN THIS PIECE OF CODE...IF YOU DON'T YOU WILL NOT UNDERSTAND THIS!!!!!!!
// READ EVERY SINGLE WORD IN THIS PIECE OF CODE...IF YOU DON'T YOU WILL NOT UNDERSTAND THIS!!!!!!!

// Open up the document in START -> WinAVR -> AVR LibC -> User Manual -> avr/interrupt.h
// Chapter 15, in Full Manual... THIS HAS A LOT OF IMPORTANT INFO...I have mentioned this at least 3 times!!!

// For those that are still having major problems, I've seen about 1/3 of the class with major problems in
// code structure. If you are still having major problems with your code, it's time to do a VERY quick overhaul.
// I've provided a skeleton structure with an example using two input capture interrupts on PORTDA0 and A3
// Please try this in the debugger.

// Create a watch variable on STATE. To do this right click on the variable STATE and then
// Add Watch 'STATE'. You can see how the variable changes as you click on PINDA0 or PINDA3. Note that the interrupt
// catches a rising edge. You modify this to suit your needs.

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>

// Global Variable
volatile char STATE;

// DEFINITIONS*

// ADC setup
volatile uint8_t sensor_value = 0;

// MOTORS-
// DC Motor

// Stepper Motor

// SENSORS-

// Optical Sensor OI
// Purpose: detect if object has reached ferromagnetic sensor and keep track of Cylinders at entrance
volatile uint8_t OI_Counter = 0; // Counts # of parts detected with IO sensor

volatile uint8_t Entry_Flag = 0; // Interrupt Flag

// Ferromagnetic Sensor IND
// Purpose: Determines if object is metallic or not, activated from OI Sensor
// Sensor is active low, meaning falling edge interrupt

volatile uint8_t IND_Type = 0; // if = 0 - non metallic if = 1 - metallic
volatile uint8_t IND_Flag = 0; // Interrupt flag

// Optical Sensor OR
// Purpose: Detect if object is in reflective sensor
volatile uint8_t OR_Flag = 0; // Interrupt Flag

// ADC definition (Reflective Sensor)

// Optical Sensor EX

// Function Definitions
void adc_init(void);
void pwmTimer(void);

int main(int argc, char *argv[])
{

    CLKPR = 0x80;
    CLKPR = 0x01; //  sets system clock to 8MHz

    STATE = 0;

    cli(); // Disables all interrupts
    // configure hardware here
    pwmTimer();

    // ADC setup
    adc_init();

    DDRD = 0b11110000; // Going to set up INT2 & INT3 on PORTD
    DDRC = 0xFF;       // just use as a display

    // Set up the Interrupt 0,3 options
    // External Interrupt Control Register A - EICRA (pg 110 and under the EXT_INT tab to the right
    // Set Interrupt sense control to catch a rising edge

    EICRA |= _BV(ISC01) | _BV(ISC00); // INT0 PD0 OI Interrupt
    EICRA |= _BV(ISC11);
    EICRA &= ~_BV(ISC10);             // INT1 PD1 IND Interrupt, falling edge
    EICRA |= _BV(ISC21) | _BV(ISC20); // INT2 PD2, OR Interrupt entry
    EICRA |= _BV(ISC31);
    EICRA &= ~_BV(ISC30); // INT3 PD3, falling edge OR exit

    //	EICRA &= ~_BV(ISC21) & ~_BV(ISC20); /* These lines would undo the above two lines */
    //	EICRA &= ~_BV(ISC31) & ~_BV(ISC30); /* Nice little trick */

    // See page 112 - EIFR External Interrupt Flags...notice how they reset on their own in 'C'...not in assembly

    EIMSK |= (_BV(INT0) | _BV(INT1) | _BV(INT2) | _BV(INT3)); // Enables INT0,INT1,INT2,INT3

    // Enable all interrupts
    sei(); // Note this sets the Global Enable for all interrupts

    goto POLLING_STAGE;

    // OI Sensor, Changes STATE = 1 -> activates magnetic stage
    if (Entry_Flag == 1)
    {                   // INTO activated, OI_Counter++
        Entry_Flag = 0; // reset entry flag
    }

// POLLING STATE
POLLING_STAGE:
    PORTC |= 0xF0; // Indicates this state is active
    switch (STATE)
    {
    case (0):
        goto POLLING_STAGE;
        break; // not needed but syntax is correct
    case (1):
        goto MAGNETIC_STAGE;
        break;
    case (2):
        goto REFLECTIVE_STAGE;
        break;
    case (3):
        goto BUCKET_STAGE;
        break;
    case (5):
        goto END;
    default:
        goto POLLING_STAGE;
    } // switch STATE

MAGNETIC_STAGE:
    // OI detected, reading IND (Ferromagnetic sensor)

    PORTC = 0x01; // Just output pretty lights know you made it here

    if (IND_Flag == 1)
    {
        IND_Flag = 0; // resets interrupt flag
    }

    // Reset the state variable
    STATE = 0;
    goto POLLING_STAGE;

REFLECTIVE_STAGE:
    // Do whatever is necessary HERE

    // 	if(OR_Flag == 1){
    //
    // 		OR_Flag = 0; // resets OR FLag
    // 	}

    PORTC = 0x04; // Just output pretty lights know you made it here
    // Reset the state variable
    STATE = 0;
    goto POLLING_STAGE;

BUCKET_STAGE:
    // Do whatever is necessary HERE
    PORTC = 0x08;
    // Reset the state variable
    STATE = 0;
    goto POLLING_STAGE;

END:
    // The closing STATE ... how would you get here?
    PORTC = 0xF0; // Indicates this state is active
    // Stop everything here...'MAKE SAFE'
    return (0);
}

// ISR's

// INT0 OI Sensor
ISR(INT0_vect)
{

    OI_Counter++;   //+1 Cylinder Count
    Entry_Flag = 1; // Activates Entry Flag for main loop
    STATE = 1;
}

// INT1 IND Sensor
ISR(INT1_vect)
{

    IND_Type = 1; // if enters here, cylinder musty be metallic
    IND_Flag = 1; // Activates interrupt flag for main
    STATE = 2;
}
/* Set up the External Interrupt 2 Vector */
ISR(INT2_vect)
{
    /* Toggle PORTC bit 2 */
    OR_Flag = 1;         // Enables Flag
    ADCSRA |= _BV(ADSC); // Starts ADC Conversion, moves to ADC Interrupt
    // STATE = 3; //Moves onto next state
}

ISR(INT3_vect)
{
    /* Toggle PORTC bit 3 */
    OR_Flag = 0; // leaves sensor
    STATE = 3;
}

ISR(ADC_vect)
{
    // adc conversion code here

    sensor_value = ADCH; // read the result

    // Do whatever you need with it (store, process, etc.)

    if (OR_Flag == 1)
    {
        ADCSRA |= _BV(ADSC); // start the next conversion immediately
    }
}

// If an unexpected interrupt occurs (interrupt is enabled and no handler is installed,
// which usually indicates a bug), then the default action is to reset the device by jumping
// to the reset vector. You can override this by supplying a function named BADISR_vect which
// should be defined with ISR() as such. (The name BADISR_vect is actually an alias for __vector_default.
// The latter must be used inside assembly code in case <avr/interrupt.h> is not included.
ISR(BADISR_vect)
{
    // user code here
}

// mTimer
void mTimer(int count)
{
    int i = 0;
    TCCR1B |= _BV(WGM12);
    OCR1A = 0x03E8; // 1 ms
    TCNT1 = 0x0000;
    // TIMSK1 |= 0b00000010;
    TIFR1 |= _BV(OCF1A);

    while (i < count)
    {
        if ((TIFR1 & 0x02) == 0x02)
        {
            TIFR1 |= _BV(OCF1A);
            i++;
        }
    }
    return;
}

// PWM
void pwmTimer()
{

    // Set Timer0 to Fast PWM mode (WGM00 + WGM01 = 1)

    TCCR0A |= (1 << WGM00) | (1 << WGM01);
    TCCR0B |= (0 << WGM02); // WGM02 = 0 (part of Fast PWM setup)

    // Enable compare match interrupt

    // TIMSK0 |= (1 << OCIE0A);

    // Set non-inverting mode on OC0A (clear on compare, set at bottom)

    TCCR0A |= (1 << COM0A1) | (0 << COM0A0);

    // Set prescaler

    TCCR0B |= (1 << CS01) | (1 << CS00);

    OCR0A = 128; // Set duty cycle

    DDRB |= (1 << PB7); // Set PB7 (OC0A) as output for PWM
}

// ADC

// initilaize adc with general channel
void adc_init(void)
{
    // Vref = AVcc, left adjust result, channel ADC0
    ADMUX = _BV(REFS0) | _BV(ADLAR); // REFS0=1 (AVcc), ADLAR=1
    ADMUX &= 0xF0;                   // MUX[3:0]=0000 (ADC0)

    // Free-running disabled; we'll retrigger in ISR (simple & robust)
    ADCSRA = _BV(ADEN) | _BV(ADIE); // enable ADC + interrupt
    ADCSRB = 0x00;

    // Prescaler /64 -> 8 MHz / 64 = 125 kHz ADC clock
    ADCSRA |= _BV(ADPS2) | _BV(ADPS1); // 110 = ÷64
}
