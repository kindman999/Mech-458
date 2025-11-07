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
#include "lcd.h"
#include "LinkedQueue.h"
#include <stdlib.h>

// Global Variable
volatile char STATE;

// DEFINITIONS*

// ADC setup

// MOTORS-
// DC Motor
#define MOTOR_DIR_PORT PORTL
#define MOTOR_DIR_DDR DDRL
#define MOTOR_PIN_IN1 (1 << PL7) // motor input A
#define MOTOR_PIN_IN2 (1 << PL6) // motor input B
#define MOTOR_ENA_PORT PORTB
#define MOTOR_ENA_DDR DDRB
#define MOTOR_ENA_PIN_A (1 << PB4) // EA
#define MOTOR_ENA_PIN_B (1 << PB5) // EB

uint8_t motor_direction_cw = 1; // 1 = CW, 0 = CCW
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
volatile uint8_t reflective_value = 0;       // reflective sensor reading
volatile uint8_t ADC_result_flag = 0;        // ADC vect flag, tells us when we are all done the adc conversions
volatile uint8_t MIN_reflective_value = 255; // minimum reflective sensor reading per object, set to max right now
volatile uint8_t sample_ready = 0;           // tells program 1 sample has finished in reflective sensor
volatile uint8_t Reflective_Counter = 0;     // counts the number of objects that have reached the reflective sensor

volatile uint8_t OBJ_Type = 0;
// 1- Ferromagnetic | 2 - Reflective | 3 - Ferromagnetic and Reflective | 4 - None
volatile uint8_t OBJ_Types[48]; // store object types
// Optical Sensor EX

// END STATE BUTTON
volatile uint8_t END_Flag = 1;
volatile uint8_t Type_1 = 0; // counters
volatile uint8_t Type_2 = 0;
volatile uint8_t Type_3 = 0;
volatile uint8_t Type_4 = 0;

#define OR_SENSOR_PIN PD2
#define OR_SENSOR_PORT PIND

// Function Definitions
void adc_init(void);
void pwmTimer(void);
void mTimer(int);
void motor_init(void);
static inline void motor_apply_direction(void);

int main(int argc, char *argv[])
{

    CLKPR = 0x80;
    CLKPR = 0x01; //  sets system clock to 8MHz

    // FIFO Setup
    link *head, *tail; // pointers to head and tail of link
    setup(&head, &tail);
    link *newlink;
    link *deQueuedLink;
    STATE = 0;

    cli(); // Disables all interrupts
    // configure hardware here
    pwmTimer();

    // LCD Setup
    InitLCD(LS_BLINK | LS_ULINE);
    LCDClear();

    // ADC setup
    adc_init();

    // start dc
    motor_init();

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

    LCDWriteInt(OI_Counter, 1); // testing OI sensor code

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
        // now add object to FIFO
        initLink(&newlink); // makes memory for the link
        newlink->e.Obj_num = OI_Counter;
        newlink->e.Ferromagetic = IND_Type;
        newlink->e.Reflective = 0; // point to 0 for now
        newlink->e.OBJ_Type = 0;
        enqueue(&head, &tail, &newlink);

        IND_Type = 0; // reset for next;
    }

    // Reset the state variable
    STATE = 0;
    goto POLLING_STAGE;

REFLECTIVE_STAGE:
    // Do whatever is necessary HERE
    while ((ADC_result_flag == 0))
    { // sits until ADC result flag reads 1 (conversions finished)

        if (sample_ready == 1)
        {
            LCDWriteInt(reflective_value, 3); // to see what reflective value is
            sample_ready = 0;
        }
    }

    ADC_result_flag = 0; // Resets ADC result flag once it finished converting all adc

    // LCDWriteInt(MIN_reflective_value,3); //display the objects total reflective value

    // categorize material type here
    // Just placeholder numbers for now
    if (MIN_reflective_value < 150)
    {
        // means object is reflective

        if (head->e.Ferromagetic == 1)
        { // check if current was ferromagnetic
            // object is reflective and ferromagnetic
            OBJ_Type = 3;
        }
        else
        {
            // object is reflective but not ferromagnetic
            OBJ_Type = 2;
        }
    }
    else
    {
        // means object is not reflective
        if (head->e.Ferromagetic == 1)
        { // check if current was ferromagnetic
            // object is not reflective and ferromagnetic
            OBJ_Type = 1;
        }
        else
        {
            // object is not reflective but not ferromagnetic
            OBJ_Type = 4;
        }
    }

    if (!isEmpty(&head))
    { // edit objects reflection value and type
        head->e.Reflective = MIN_reflective_value;
        head->e.OBJ_Type = OBJ_Type;
    }
    // LCDWriteInt(OBJ_Type,1); //display the object type

    // add info to array
    OBJ_Types[OI_Counter - 1] = head->e.OBJ_Type;
    OBJ_Type = 0;                  // reset obj typ
    dequeue(&head, &deQueuedLink); // dequeue head
    free(deQueuedLink);            // free space

    // SORTING FUNCTION CALLED HERE

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
    if (END_Flag == 1)
    {
        END_Flag = 0;
    }

    OCR0A = 0; // stops dc motor

    for (int i = 0; i < OI_Counter; i++)
    {
        int FINAL_OBJ = OBJ_Types[i];
        switch (FINAL_OBJ)
        {
        case 0: // None / glitch
            Type_1++;
            break;
        case 1: // Reflective
            Type_2++;
            break;
        case 2: // Ferromagnetic + Reflective
            Type_3++;
            break;
        case 3: // Non-ferro, non-reflective
            Type_4++;
            break;
        default:
            // optional error handling
            break;
        }
    }

    LCDClear();
    LCDWriteString("Type 1: ");
    LCDWriteInt(Type_1, 2);
    mTimer(1000); // wait 1 second

    LCDClear();
    LCDWriteString("Type 2: ");
    LCDWriteInt(Type_2, 2);
    mTimer(1000); // wait 1 second

    LCDClear();
    LCDWriteString("Type 3: ");
    LCDWriteInt(Type_3, 2);
    mTimer(1000); // wait 1 second

    LCDClear();
    LCDWriteString("Type 4: ");
    LCDWriteInt(Type_4, 2);
    mTimer(1000); // wait 1 second

    return (0);
}

// ISR's

// INT0 OI Sensor
ISR(INT0_vect)
{
    mTimer(5);      // debounce
    OI_Counter++;   //+1 Cylinder Count
    Entry_Flag = 1; // Activates Entry Flag for main loop
    STATE = 1;
}

// INT1 IND Sensor
ISR(INT1_vect)
{
    mTimer(5);    // debounce
    IND_Type = 1; // if enters here, cylinder musty be metallic
    IND_Flag = 1; // Activates interrupt flag for main
}
/* Set up the External Interrupt 2 Vector */
ISR(INT2_vect)
{
    /* Toggle PORTC bit 2 */
    // OR_Flag = 1; //Enables Flag
    mTimer(5); // debounce
    STATE = 2;
    reflective_value = 255;     // resets reflective value
    MIN_reflective_value = 255; // resets minimum reflective value

    ADCSRA |= _BV(ADSC); // Starts ADC Conversion, moves to ADC Interrupt
}

ISR(INT3_vect)
{
    /* Toggle PORTC bit 3 */
    // end state temporary
    mTimer(5); // debounce
    END_Flag = 1;
    STATE = 5;
}

ISR(ADC_vect)
{
    // adc conversion code here
    //  Do whatever you need with it (store, process, etc.)

    if (OR_SENSOR_PORT & (1 << OR_SENSOR_PIN))
    {
        reflective_value = ADCH; // read the

        if (reflective_value < MIN_reflective_value)
        {                                            // checks if it is new minimum
            MIN_reflective_value = reflective_value; // sets it as new minimum
        }

        sample_ready = 1;
        ADCSRA |= _BV(ADSC); // start the next conversion immediately
    }
    else
    {

        ADC_result_flag = 1; // ADC ended go back to main
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

void motor_init(void)
{
    // Direction pins
    MOTOR_DIR_DDR |= MOTOR_PIN_IN1 | MOTOR_PIN_IN2;

    // Enable pins
    MOTOR_ENA_DDR |= MOTOR_ENA_PIN_A | MOTOR_ENA_PIN_B;

    // Enable EA/EB
    MOTOR_ENA_PORT |= MOTOR_ENA_PIN_A | MOTOR_ENA_PIN_B;

    // Default direction CW: IN1=1, IN2=0
    motor_apply_direction();
}
static inline void motor_apply_direction(void)
{
    if (motor_direction_cw)
    {
        MOTOR_DIR_PORT |= MOTOR_PIN_IN1;
        MOTOR_DIR_PORT &= ~MOTOR_PIN_IN2;
    }
    else
    {
        MOTOR_DIR_PORT &= ~MOTOR_PIN_IN1;
        MOTOR_DIR_PORT |= MOTOR_PIN_IN2;
    }
}
