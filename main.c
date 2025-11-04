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

// Global Variable
volatile char STATE;

int main(int argc, char *argv[])
{

    CLKPR = 0x80;
    CLKPR = 0x01; //  sets system clock to 8MHz

    STATE = 0;

    cli(); // Disables all interrupts

    DDRD = 0b11110000; // Going to set up INT2 & INT3 on PORTD
    DDRC = 0xFF;       // just use as a display

    // Set up the Interrupt 0,3 options
    // External Interrupt Control Register A - EICRA (pg 110 and under the EXT_INT tab to the right
    // Set Interrupt sense control to catch a rising edge
    EICRA |= _BV(ISC21) | _BV(ISC20);
    EICRA |= _BV(ISC31) | _BV(ISC30);

    //	EICRA &= ~_BV(ISC21) & ~_BV(ISC20); /* These lines would undo the above two lines */
    //	EICRA &= ~_BV(ISC31) & ~_BV(ISC30); /* Nice little trick */

    // See page 112 - EIFR External Interrupt Flags...notice how they reset on their own in 'C'...not in assembly
    EIMSK |= 0x0C;

    // Enable all interrupts
    sei(); // Note this sets the Global Enable for all interrupts

    goto POLLING_STAGE;

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
    // Do whatever is necessary HERE
    PORTC = 0x01; // Just output pretty lights know you made it here
    // Reset the state variable
    STATE = 0;
    goto POLLING_STAGE;

REFLECTIVE_STAGE:
    // Do whatever is necessary HERE
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

/* Set up the External Interrupt 2 Vector */
ISR(INT2_vect)
{
    /* Toggle PORTC bit 2 */
    STATE = 2;
}

ISR(INT3_vect)
{
    /* Toggle PORTC bit 3 */
    STATE = 3;
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
