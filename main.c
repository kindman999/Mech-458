#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "lcd.h"
#include "LinkedQueue.h"
#include "drivers.h" // Include the new drivers file

// --- GLOBAL VARIABLES ---
volatile char STATE = 0;

// Stepper Motor globals
int current_step = 0;
int direction = 0;					   // 1 - CW, 0 - CCW
volatile uint8_t stepper_position = 4; // 0-alum, 2-steel, 3-white, 4-black
volatile uint8_t stepper_flag = 0;	   // 1 means object must be sorted
volatile uint8_t sorted_flag = 1;	   // 1 means sorter is ready
volatile int step_count = 0;

volatile uint16_t stepper_steps_left = 0; // How many steps remaining
volatile int stepper_dir_request = 0;	  // 1=CW, 0=CCW
volatile int steps_moved_so_far = 0;	  // For S-Curve calculation

// Stepper S-curve control
volatile uint8_t stepper_scurve_active = 0;
volatile uint16_t stepper_total_steps = 0;
volatile uint16_t stepper_step_index = 0;

// Sensor globals
volatile uint8_t OI_Counter = 0;
volatile uint8_t Entry_Flag = 0;
volatile uint8_t OR_Flag = 0;

// ADC / Reflective globals
volatile uint16_t reflective_value = 0;
volatile uint16_t ADC_result_flag = 0;
volatile uint16_t MIN_reflective_value = 1023;
volatile uint8_t sample_ready = 0;
volatile uint8_t Reflective_Counter = 0;

volatile uint8_t OBJ_Type = 0;
volatile uint8_t OBJ_Types[100]; // store object types

// Exit / HE / Stop globals
volatile uint8_t EX_Flag = 0;
element Test;
element Sort_Element;
volatile uint8_t HE_Flag = 0;
volatile uint8_t END_Flag = 1;
volatile uint8_t Type_1 = 0;
volatile uint8_t Type_2 = 0;
volatile uint8_t Type_3 = 0;
volatile uint8_t Type_4 = 0;
volatile uint8_t stop_request_flag = 0;

volatile uint8_t conveyor_hold_for_stepper = 0;      // request to stop at exit
volatile uint8_t conveyor_stopped_for_stepper = 0;   // conveyor currently held


// --- FUNCTION PROTOTYPES FOR LOCAL LOGIC ---
void step(int);
void step_zero(void);
void sort(int);

int main(int argc, char *argv[])
{
	CLKPR = 0x80;
	CLKPR = 0x01; // sets system clock to 8MHz
	TCCR1B |= _BV(CS11);

	// FIFO Setup
	link *head, *tail;
	setup(&head, &tail);
	link *newlink;
	link *deQueuedLink;

	STATE = 0;

	cli(); // Disables all interrupts

	// Hardware Setup
	pwmTimer();
	adc_init();
	motor_init();
	InitLCD(LS_BLINK | LS_ULINE);
	LCDClear();

	// Stepper Pins
	DDRA = 0xFF;
	PORTA = 0x00;

	// Interrupt Pins
	DDRD = 0b11110000;
	DDRC = 0xFF;

	// Interrupt Configuration
	EICRA |= _BV(ISC01) | _BV(ISC00); // INT0 Rising (OI)
	EICRA |= _BV(ISC11) | _BV(ISC10); // INT1 Rising (EX)
	EICRA &= ~(_BV(ISC21) | _BV(ISC20));
	EICRA |= _BV(ISC21);			  // INT2 Falling (HE)
	EICRA |= _BV(ISC31) | _BV(ISC30); // INT3 Rising (Stop)

	EIMSK |= (_BV(INT0) | _BV(INT1) | _BV(INT2) | _BV(INT3));

	sei(); // Global Enable

	// Startup Sequence
	motor_scurve_accel(0, 80, 400, 40);
	step_zero();

	// --- MAIN WHILE LOOP (Replaces Goto) ---
	while (1)
	{

		// stepper, always passive
		if (stepper_steps_left > 0)
		{
			// A. S-Curve Math Setup
			const float max_delay = 12.0f; // Slowest speed (start/stop)
			const float min_delay = 2.0f;  // Fastest speed (middle)
			const int ramp_steps = 15;	   // How many steps to accelerate

			float delay_ms = max_delay;

			// Calculate Progress
			if (steps_moved_so_far < ramp_steps)
			{
				// ACCELERATION PHASE
				float t = (float)steps_moved_so_far / (float)ramp_steps;
				float s = t * t * (3.0f - 2.0f * t); // Smoothstep equation
				delay_ms = max_delay - ((max_delay - min_delay) * s);
			}
			else if (stepper_steps_left <= ramp_steps)
			{
				// DECELERATION PHASE
				float t = (float)stepper_steps_left / (float)ramp_steps;
				float s = t * t * (3.0f - 2.0f * t);
				delay_ms = max_delay - ((max_delay - min_delay) * s);
			}
			else
			{
				// Full Speed
				delay_ms = min_delay;
			}

			if (delay_ms < min_delay)
				delay_ms = min_delay;

			// EXECUTE THE STEP
			step(stepper_dir_request); // Moves instantly

			// delay
			mTimer((int)(delay_ms + 0.5f));

			// F. Update Counters
			stepper_steps_left--;
			steps_moved_so_far++;
			if(stepper_steps_left == 0){
				stepper_scurve_active = 0;
			}
		}

		// POLLING LOGIC
		if (STATE == 0)
		{

        // ----------------------------------------------------
        // Hold conveyor at exit until stepper is fully sorted
        // ----------------------------------------------------
        if (conveyor_hold_for_stepper && !conveyor_stopped_for_stepper)
        {
            // Smoothly stop the conveyor at the exit sensor
            uint8_t current_duty = OCR0A;
            motor_scurve_decel(current_duty, 0, 300, 30); // tune 300 ms / 30 steps if needed
            conveyor_stopped_for_stepper = 1;
        }

        // If we're stopped for the stepper and the stepper is now done,
        // restart the conveyor.
        if (conveyor_stopped_for_stepper && !stepper_scurve_active)
        {
            // Resume to your normal run speed (80 here)
            motor_scurve_accel(0, 80, 300, 30);           // tune as needed
            conveyor_stopped_for_stepper = 0;
            conveyor_hold_for_stepper = 0;
        }

			// Stop Request
			if (stop_request_flag)
			{
				uint8_t current_duty = OCR0A;
				motor_scurve_decel(current_duty, 0, 1000, 50);
				stop_request_flag = 0;
				STATE = 4;
			}

			// Sorting Logic, only enters if object through reflective, previous object sorted, and the sort has zero steps left
			if (stepper_flag == 1 && sorted_flag == 1 && stepper_steps_left == 0)
			{
				sort(head->e.OBJ_Type);
				sorted_flag = 0;
				stepper_flag = 0;
			}

			// State Transitions
			if (STATE != 4)
			{

				if (EX_Flag == 1)
				{
					STATE = 3; // Bucket
				}
				else if (Entry_Flag == 1)
				{
					STATE = 2; // OR Sensor
					Entry_Flag = 0;
				}
				else
				{
					STATE = 0; // Stay in polling nothing happening yet
				}
			}
		}

		// STATE MACHINE SWITCH
		switch (STATE)
		{
		case 0: // POLLING
			// Logic handled in the 'if' block above for continuous checking
			break;

		case 1:
			STATE = 0;
			break;

		case 2: // REFLECTIVE STAGE
			// if adc is not finished, go bacl to main loop
			if (ADC_result_flag == 0)
			{
				break;
			}

			if (MIN_reflective_value >= 0 && MIN_reflective_value < 250)
			{
				OBJ_Type = 1; // Aluminum
			}
			else if (MIN_reflective_value >= 250 && MIN_reflective_value < 600)
			{
				OBJ_Type = 2; // Steel
			}
			else if (MIN_reflective_value >= 600 && MIN_reflective_value < 970)
			{
				OBJ_Type = 3; // White
			}
			else if (MIN_reflective_value >= 970 && MIN_reflective_value <= 1023)
			{
				OBJ_Type = 4; // Black
			}

			// Enqueue
			initLink(&newlink);
			newlink->e.Obj_num = OI_Counter;
			newlink->e.Reflective = MIN_reflective_value;
			newlink->e.OBJ_Type = OBJ_Type;
			enqueue(&head, &tail, &newlink);

			stepper_flag = 1;
			ADC_result_flag = 0;

			STATE = 0;
			break;

		case 3: // BUCKET STAGE

			// see if stepper has reached certain point in sort, if not stop
			if (stepper_steps_left > 15)
			{
				motor_set_speed(0);
				break;
			}

			// sorted belt resumes
			motor_set_speed(80);

			if (EX_Flag == 1)
			{
				EX_Flag = 0;
			}

			Test = firstValue(&head);
			int Current_OBJ_Type = Test.OBJ_Type;
			int Current_OBJ_Num = Test.Obj_num;
			uint16_t Current_Reflective = Test.Reflective;

			OBJ_Types[Current_OBJ_Num - 1] = Current_OBJ_Type;

			LCDClear();
			LCDWriteString("Type:");
			LCDWriteInt(Current_OBJ_Type, 1);
			LCDWriteString(" #:");
			LCDWriteInt(Current_OBJ_Num, 2);
			LCDGotoXY(0, 1);
			LCDWriteString("RF:");
			LCDWriteInt(Current_Reflective, 4);

			dequeue(&head, &tail, &deQueuedLink);
			free(deQueuedLink);

			sorted_flag = 1;

			STATE = 0;
			break;

		case 4: // END
			for (int i = 0; i < OI_Counter; i++)
			{
				int FINAL_OBJ = OBJ_Types[i];
				switch (FINAL_OBJ)
				{
				case 1:
					Type_1++;
					break;
				case 2:
					Type_2++;
					break;
				case 3:
					Type_3++;
					break;
				case 4:
					Type_4++;
					break;
				}
			}

			LCDClear();
			LCDWriteString("Aluminum: ");
			LCDWriteInt(Type_1, 2);
			mTimer(2000);
			LCDClear();
			LCDWriteString("Steel: ");
			LCDWriteInt(Type_2, 2);
			mTimer(2000);
			LCDClear();
			LCDWriteString("White: ");
			LCDWriteInt(Type_3, 2);
			mTimer(2000);
			LCDClear();
			LCDWriteString("Black: ");
			LCDWriteInt(Type_4, 2);
			mTimer(2000);

			while (1)
				; // Stop here forever
			break;
		}
	}
	return 0;
}

// --- LOCAL LOGIC FUNCTIONS ---

void step(int direction)
{
	// Update the Step Counter
	if (direction == 1)
	{ // CW
		current_step++;
		if (current_step > 4)
			current_step = 1;
	}
	else
	{ // CCW
		current_step--;
		if (current_step < 1)
			current_step = 4;
	}

	// Update the Pins
	switch (current_step)
	{
	case (1):
		PORTA = 0b00010111;
		break;
	case (2):
		PORTA = 0b00011011;
		break;
	case (3):
		PORTA = 0b00101011;
		break;
	case (4):
		PORTA = 0b00100111;
		break;
	}
}

void step_zero(void)
{
	HE_Flag = 0;
	stepper_position = 4;
	while (1)
	{
		step(direction);
		mTimer(10);
		if (HE_Flag == 1)
		{
			mTimer(20);
			HE_Flag = 0;
			break;
		}
	}
}

void sort(int OBJ_Type)
{
	LCDClear();
	LCDWriteInt(stepper_position, 1);
	LCDWriteInt(OBJ_Type, 1);
	step_count = 0;

	// --- LOGIC TO DETERMINE PATH (Same as before) ---
	if (OBJ_Type == 1)
	{ // Aluminum
		switch (stepper_position)
		{
		case (1):
			stepper_position = 1;
			break;
		case (2):
			direction = 1;
			step_count = 100;
			stepper_position = 1;
			break;
		case (3):
			direction = 1;
			step_count = 50;
			stepper_position = 1;
			break;
		case (4):
			direction = 0;
			step_count = 50;
			stepper_position = 1;
			break;
		}
	}
	else if (OBJ_Type == 2)
	{ // Steel
		switch (stepper_position)
		{
		case (1):
			direction = 0;
			step_count = 100;
			stepper_position = 2;
			break;
		case (2):
			stepper_position = 2;
			break;
		case (3):
			direction = 0;
			step_count = 50;
			stepper_position = 2;
			break;
		case (4):
			direction = 1;
			step_count = 50;
			stepper_position = 2;
			break;
		}
	}
	else if (OBJ_Type == 3)
	{ // White
		switch (stepper_position)
		{
		case (1):
			direction = 0;
			step_count = 50;
			stepper_position = 3;
			break;
		case (2):
			direction = 1;
			step_count = 50;
			stepper_position = 3;
			break;
		case (3):
			stepper_position = 3;
			break;
		case (4):
			direction = 1;
			step_count = 100;
			stepper_position = 3;
			break;
		}
	}
	else if (OBJ_Type == 4)
	{ // Black
		switch (stepper_position)
		{
		case (1):
			direction = 1;
			step_count = 50;
			stepper_position = 4;
			break;
		case (2):
			direction = 0;
			step_count = 50;
			stepper_position = 4;
			break;
		case (3):
			direction = 0;
			step_count = 100;
			stepper_position = 4;
			break;
		case (4):
			stepper_position = 4;
			break;
		}
	}

	// --- NEW: ASSIGN THE JOB TO MAIN LOOP ---
	// Instead of looping here, we just set these variables.
	// The Main Loop will see 'stepper_steps_left > 0' and start moving.

	if (step_count > 0)
	{
		stepper_dir_request = direction;
		stepper_steps_left = step_count;
		steps_moved_so_far = 0; // Reset the S-Curve counter
		stepper_scurve_active = 1;
	}
}

// --- ISRs ---

ISR(INT0_vect)
{
	Entry_Flag = 1;
	reflective_value = 0;
	MIN_reflective_value = 1023;
	OI_Counter++;
	ADCSRA |= _BV(ADSC);
}

ISR(INT1_vect)
{
	EX_Flag = 1;
	if(stepper_scurve_active){
		conveyor_hold_for_stepper = 1;
	}
}

ISR(INT2_vect)
{
	HE_Flag = 1;
}

ISR(INT3_vect)
{
	stop_request_flag = 1;
}

ISR(ADC_vect)
{
	reflective_value = ADC;
	if (reflective_value < MIN_reflective_value)
	{
		MIN_reflective_value = reflective_value;
	}

	if (OR_SENSOR_PORT & (1 << OR_SENSOR_PIN))
	{
		ADCSRA |= _BV(ADSC);
	}
	else
	{
		ADC_result_flag = 1;
	}
}

ISR(BADISR_vect)
{
	// user code here
}
