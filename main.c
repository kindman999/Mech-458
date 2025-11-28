#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "lcd.h"
#include "LinkedQueue.h"
#include "drivers.h" // Include the new drivers file

// --- STEPPER S-CURVE TABLE (INT-ONLY) ---
#define STEPPER_RAMP_STEPS 16
// Index 0 = slowest (start/stop), index 12 = fastest (middle / cruise)
const uint8_t stepper_delay_table[STEPPER_RAMP_STEPS + 1] =
	{
		18, // 0 - very slow
		18, // 1
		17, // 2
		17, // 3
		16, // 4
		14, // 5
		13, // 6
		13, // 7
		11, // 8
		10, // 9
		9,	// 10
		8,	// 11
		7,	// 12
		7,	// 13
		6,	// 14
		6,	// 15
		6	// 16 - fastest
};

// --- GLOBAL VARIABLES ---
volatile char STATE = 0;

// Stepper Motor globals
int current_step = 0;
int direction = 0;					   // 1 - CW, 0 - CCW
volatile uint8_t stepper_position = 4; // 0-alum, 2-steel, 3-white, 4-black
volatile uint8_t stepper_flag = 0;	   // 1 means object must be sorted
volatile uint8_t sorted_flag = 0;	   // 1 means sorter is ready
volatile int step_count = 0;

volatile uint16_t stepper_steps_left = 0; // How many steps remaining
volatile int stepper_dir_request = 0;	  // 1=CW, 0=CCW
volatile int steps_moved_so_far = 0;	  // For S-Curve calculation
volatile int same_object_flag = 0;
// Stepper S-curve control
// volatile uint8_t stepper_scurve_active = 0; unused variable
volatile uint16_t stepper_total_steps = 0;
// volatile uint16_t stepper_step_index = 0; unused variable

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
volatile uint8_t RETURN_TO_SCAN = 0;

volatile uint8_t OBJ_Type = 0;
volatile uint8_t OBJ_Types[100];  // store object types
volatile uint8_t OBJ_Types2[100]; // store object types, still on belt

// Exit / HE / Stop globals
volatile uint8_t EX_Flag = 0;
volatile uint8_t EX_Count = 0;
element Test;
element Test2;
element Sort_Element;
volatile uint8_t HE_Flag = 0;
volatile uint8_t END_Flag = 1;
volatile uint8_t Type_1 = 0;
volatile uint8_t Type_2 = 0;
volatile uint8_t Type_3 = 0;
volatile uint8_t Type_4 = 0;
volatile uint8_t Type_12 = 0;
volatile uint8_t Type_22 = 0;
volatile uint8_t Type_32 = 0;
volatile uint8_t Type_42 = 0;
volatile uint8_t stop_request_flag = 0;

// PAUSE BUTTON GLOBALS ---
volatile uint8_t pause_request_flag = 0; // Set by pause button ISR
volatile uint8_t pause_active = 0;		 // Prevent re-entrancy
volatile uint8_t saved_duty_cycle = 0;	 // Store OCR0A before pausing
volatile uint8_t system_paused = 0;

// ramp down
volatile int rampdown_flag = 0;
volatile int Total_Sorted_Count = 0;

// --- FUNCTION PROTOTYPES FOR LOCAL LOGIC ---
void step(int);
void step_zero(void);
void sort(int);
void motor_stop(void);

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

	// PAUSE BUTTON PIN SETUP (PE4 / INT4) ---
	DDRE &= ~(1 << PE4); // PE4 as input
	PORTE |= (1 << PE4); // Enable pull-up on pause button

	// Interrupt Configuration
	EICRA |= _BV(ISC01) | _BV(ISC00); // INT0 Rising (OI)
	EICRA |= _BV(ISC11) | _BV(ISC10); // INT1 Rising (EX)
	EICRA &= ~(_BV(ISC21) | _BV(ISC20));
	EICRA |= _BV(ISC21);			  // INT2 Falling (HE)
	EICRA |= _BV(ISC31) | _BV(ISC30); // INT3 Rising (Stop)

	// PAUSE BUTTON INTERRUPT CONFIG (INT4, falling edge) ---
	EICRB &= ~(_BV(ISC40)); // ISC41:40 = 10 -> falling edge
	EICRB |= _BV(ISC41);	// INT4 Falling (Pause)

	EIMSK |= (_BV(INT0) | _BV(INT1) | _BV(INT2) | _BV(INT3) | _BV(INT4));

	sei(); // Global Enable

	// Startup Sequence
	motor_scurve_accel(0, 80, 400, 40);
	step_zero();

	// --- MAIN WHILE LOOP (Replaces Goto) ---
	while (1)
	{
		// stepper
		if (stepper_steps_left > 0)
		{
			uint8_t delay_ms;
			uint8_t idx;

			// ACCELERATION PHASE
			if (steps_moved_so_far < STEPPER_RAMP_STEPS)
			{
				idx = (uint8_t)steps_moved_so_far;
			}
			// DECELERATION PHASE
			else if (stepper_steps_left <= STEPPER_RAMP_STEPS)
			{
				idx = (uint8_t)stepper_steps_left;
			}
			// CRUISE PHASE
			else
			{
				idx = STEPPER_RAMP_STEPS;
			}

			if (idx > STEPPER_RAMP_STEPS)
			{
				idx = STEPPER_RAMP_STEPS;
			}

			delay_ms = stepper_delay_table[idx];

			step(stepper_dir_request); // Execute one step
			mTimer(delay_ms);		   // Integer delay from table

			stepper_steps_left--;
			steps_moved_so_far++;
		}
		// PAUSE HANDLING (TOGGLE PAUSE / RESUME)
		if (pause_request_flag && !pause_active)
		{
			// Consume the request from ISR
			pause_request_flag = 0;
			pause_active = 1;

			// Disable INT4 while we are handling pause/resume to avoid extra toggles
			EIMSK &= ~_BV(INT4);

			mTimer(30);

			// wait until button is released again, PE4 is pulled up
			while (!(PINE & (1 << PE4)))
			{
				// button held down
			}
			if (!system_paused)
			{
				// RUNNING ? go into PAUSE
				system_paused = 1;

				// Save current conveyor speed (duty cycle)
				saved_duty_cycle = OCR0A;
				if (saved_duty_cycle == 0)
				{
					// Safety fallback in case we somehow paused at 0
					saved_duty_cycle = 50; // your normal run speed
				}

				// Smooth ramp down to a stop
				/*motor_scurve_decel(saved_duty_cycle, 0, 1000, 50);*/
				motor_stop();
				OCR0A = 0; // ensure conveyor is fully stopped

				// STOP ALL MOTION: kill any in-progress stepper move
				stepper_steps_left = 0;
				steps_moved_so_far = 0;

				// Return the state machine to the idle/polling state
				STATE = 0;

				// LCD Code
				Type_1 = 0;
				Type_2 = 0;
				Type_3 = 0;
				Type_4 = 0;
				Type_12 = 0;
				Type_22 = 0;
				Type_32 = 0;
				Type_42 = 0;

				// counts how many objects of each type have been sorted into bucket
				for (int i = 0; i < OI_Counter; i++)
				{
					int FINAL_OBJ = OBJ_Types[i]; // bucketed
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

					int FINAL_OBJ2 = OBJ_Types2[i]; // reflective sensed
					switch (FINAL_OBJ2)
					{
					case 1:
						Type_12++;
						break;
					case 2:
						Type_22++;
						break;
					case 3:
						Type_32++;
						break;
					case 4:
						Type_42++;
						break;
					}
				}

				int Aluminum_belt = Type_12 - Type_1;
				int Steel_belt = Type_22 - Type_2;
				int White_belt = Type_32 - Type_3;
				int Black_belt = Type_42 - Type_4;

				LCDClear();

				// bucketd
				LCDWriteString("Bin: ");
				LCDWriteInt(Type_1, 2);
				LCDWriteString(" ");
				LCDWriteInt(Type_2, 2);
				LCDWriteString(" ");
				LCDWriteInt(Type_3, 2);
				LCDWriteString(" ");
				LCDWriteInt(Type_4, 2);

				// still on belt
				LCDGotoXY(0, 1);
				LCDWriteString("Blt: ");
				LCDWriteInt(Aluminum_belt, 2);
				LCDWriteString(" ");
				LCDWriteInt(Steel_belt, 2);
				LCDWriteString(" ");
				LCDWriteInt(White_belt, 2);
				LCDWriteString(" ");
				LCDWriteInt(Black_belt, 2);
				LCDWriteString(" ");
			}
			else
			{
				// PAUSED ? RESUME
				system_paused = 0;

				// Ramp back up to previous speeda
				motor_apply_direction();
				/*	motor_scurve_accel(0, saved_duty_cycle, 400, 40);*/
				OCR0A = saved_duty_cycle;
				LCDClear();
			}

			// Clear any pending INT4 flag and re-enable the pause interrupt
			EIFR |= _BV(INTF4); // clear INT4 flag
			EIMSK |= _BV(INT4); // re-enable INT4

			pause_active = 0;
		}

		// POLLING LOGIC
		if (STATE == 0)
		{
			// Stop Request
			if (stop_request_flag)
			{
				// 				uint8_t current_duty = OCR0A;
				// 				motor_scurve_decel(current_duty, 0, 1000, 50);
				stop_request_flag = 0;
				rampdown_flag = 1;
				LCDClear();
				LCDWriteString("RAMPING DOWN");
			}

			// rampdown condition
			if (rampdown_flag == 1)
			{
				// cjheck if the same amount of objects sorted, have been placed in bucked
				if ((OI_Counter == Total_Sorted_Count) && (stepper_steps_left == 0))
				{
					STATE = 4;
				}
			}

			// 			if ((head != NULL) && (stepper_steps_left == 0)) //premtive sort
			// 		{
			// 				if (stepper_position != head->e.OBJ_Type)
			// 				{
			//  					sort(head->e.OBJ_Type);
			//  					sorted_flag = 1;
			// 				}

			// State Transitions
			if (STATE != 4)
			{

				if (EX_Flag == 1)
				{

					if (sorted_flag == 0 || (stepper_steps_left == 0))
					{
						sort(head->e.OBJ_Type);
						sorted_flag = 1;
						EX_Flag = 0;
						EX_Count--;
					}

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

			if (EX_Flag == 1)
			{ // Remember to come back

				// Manually trigger the sort
				sort(head->e.OBJ_Type);
				sorted_flag = 1;
				EX_Flag = 0;
				EX_Count--;

				STATE = 3; // Jump to Bucket Stage
				break;
			}
			// restart belt
			if (stepper_steps_left == 0)
			{
				if (OCR0A == 0)
				{
					OCR0A = 80;
					motor_apply_direction();
				}
			}
			// ------------------------------------

			// if adc is not finished, go back to main loop
			if (ADC_result_flag == 0)
			{
				break;
			}

			if (MIN_reflective_value >= 0 && MIN_reflective_value < 250)
			{
				OBJ_Type = 1; // Aluminum
			}
			else if (MIN_reflective_value >= 250 && MIN_reflective_value < 750)
			{
				OBJ_Type = 2; // Steel
			}
			else if (MIN_reflective_value >= 750 && MIN_reflective_value < 960)
			{
				OBJ_Type = 3; // White
			}
			else if (MIN_reflective_value >= 960 && MIN_reflective_value <= 1023)
			{
				OBJ_Type = 4; // Black
			}
			// 						LCDClear();
			// 					LCDWriteInt(MIN_reflective_value,4);
			//
			// Enqueue
			initLink(&newlink);
			newlink->e.Obj_num = OI_Counter;
			newlink->e.Reflective = MIN_reflective_value;
			newlink->e.OBJ_Type = OBJ_Type;
			enqueue(&head, &tail, &newlink);

			OBJ_Types2[OI_Counter - 1] = OBJ_Type;

			stepper_flag = 1;
			ADC_result_flag = 0;

			STATE = 0;
			break;

		case 3: // BUCKET STAGE

			if (ADC_result_flag == 1)
			{

				if (MIN_reflective_value >= 0 && MIN_reflective_value < 250)
				{
					OBJ_Type = 1; // Aluminum
				}
				else if (MIN_reflective_value >= 250 && MIN_reflective_value < 750)
				{
					OBJ_Type = 2; // Steel
				}
				else if (MIN_reflective_value >= 750 && MIN_reflective_value < 960)
				{
					OBJ_Type = 3; // White
				}
				else if (MIN_reflective_value >= 960 && MIN_reflective_value <= 1023)
				{
					OBJ_Type = 4; // Black
				}
				// 					LCDClear();
				// 					LCDWriteInt(MIN_reflective_value,4);

				// Enqueue
				initLink(&newlink);
				newlink->e.Obj_num = OI_Counter;
				newlink->e.Reflective = MIN_reflective_value;
				newlink->e.OBJ_Type = OBJ_Type;
				enqueue(&head, &tail, &newlink);

				OBJ_Types2[OI_Counter - 1] = OBJ_Type;
				ADC_result_flag = 0;
				stepper_flag = 1;
			}

			// see if stepper has reached certain point in sort, if not stop
			if ((stepper_steps_left > 8) && same_object_flag == 0)
			{
				OCR0A = 0;
				break;
			}

			// sorted belt resumes

			OCR0A = 80;
			motor_apply_direction();

			if (stepper_steps_left > 0)

			{
				break; // Belt is running, but we stay in Case 3 to protect the stepper logic.
			}
			// commented out below so that the bucket stage doesn't wipe out knowledge of pneding exits anymore

			Test = firstValue(&head);
			int Current_OBJ_Type = Test.OBJ_Type;
			int Current_OBJ_Num = Test.Obj_num;
			uint16_t Current_Reflective = Test.Reflective;

			OBJ_Types[Current_OBJ_Num - 1] = Current_OBJ_Type;

			// 			LCDClear();
			// 			LCDWriteString("Type:");
			// 			LCDWriteInt(Current_OBJ_Type, 1);
			// 			LCDWriteString(" #:");
			// 			LCDWriteInt(Current_OBJ_Num, 2);
			// 			LCDGotoXY(0, 1);
			// 			LCDWriteString("RF:");
			// 			LCDWriteInt(Current_Reflective, 4);

			dequeue(&head, &tail, &deQueuedLink);
			free(deQueuedLink);
			Total_Sorted_Count++;
			sorted_flag = 0;
			same_object_flag = 0;

			STATE = 0; // Normal idle

			break;

		case 4: // END, RAMP DOWN

			motor_stop();

			Type_1 = 0;
			Type_2 = 0;
			Type_3 = 0;
			Type_4 = 0;

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
			LCDWriteString("Bin: ");
			LCDWriteInt(Type_1, 2);
			LCDWriteString(" ");
			LCDWriteInt(Type_2, 2);
			LCDWriteString(" ");
			LCDWriteInt(Type_3, 2);
			LCDWriteString(" ");
			LCDWriteInt(Type_4, 2);

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
		mTimer(15);
		if (HE_Flag == 1)
		{
			mTimer(20);
			HE_Flag = 0;
			break;
		}
	}
}

void motor_stop(void)
{
	// 	#define MOTOR_PIN_IN1  (1 << PL7) // motor input A
	// 	#define MOTOR_PIN_IN2  (1 << PL6) // motor input B
	//
	// 	#define MOTOR_ENA_PIN_A (1 << PL4) // EA
	// 	#define MOTOR_ENA_PIN_B (1 << PL5) // EB
	PORTL = 0b11110000;
}
void sort(int OBJ_Type)
{
	/*LCDClear();*/
	// 	LCDWriteInt(stepper_position, 1);
	// 	LCDWriteInt(OBJ_Type, 1);
	step_count = 0;

	// --- LOGIC TO DETERMINE PATH (Same as before) ---
	if (OBJ_Type == 1)
	{ // Aluminum
		switch (stepper_position)
		{
		case (1):
			stepper_position = 1;
			same_object_flag = 1;
			break;
		case (2):
			// direction = 1;
			step_count = 100;
			stepper_position = 1;
			break;
		case (3):
			direction = 0;
			step_count = 50;
			stepper_position = 1;
			break;
		case (4):
			direction = 1;
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
			// direction = 0;
			step_count = 100;
			stepper_position = 2;
			break;
		case (2):
			stepper_position = 2;
			same_object_flag = 1;
			break;
		case (3):
			direction = 1;
			step_count = 50;
			stepper_position = 2;
			break;
		case (4):
			direction = 0;
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
			direction = 1;
			step_count = 50;
			stepper_position = 3;
			break;
		case (2):
			direction = 0;
			step_count = 50;
			stepper_position = 3;
			break;
		case (3):
			stepper_position = 3;
			same_object_flag = 1;
			break;
		case (4):
			// direction = 1;
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
			direction = 0;
			step_count = 50;
			stepper_position = 4;
			break;
		case (2):
			direction = 1;
			step_count = 50;
			stepper_position = 4;
			break;
		case (3):
			// direction = 0;
			step_count = 100;
			stepper_position = 4;
			break;
		case (4):
			stepper_position = 4;
			same_object_flag = 1;
			break;
		}
	}

	if (step_count > 0)
	{
		stepper_dir_request = direction;
		stepper_steps_left = step_count;
		steps_moved_so_far = 0; // Reset the S-Curve counter
		stepper_total_steps = step_count;
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
	EX_Count++;
	motor_stop();
}

ISR(INT2_vect)
{
	HE_Flag = 1;
}

// INT3 Ramp down
ISR(INT3_vect)
{
	stop_request_flag = 1;
}
// INT4 Pause Button
ISR(INT4_vect)
{
	pause_request_flag = 1;
	EIFR |= _BV(INTF4);
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