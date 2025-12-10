#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "lcd.h"
#include "LinkedQueue.h"
#include "drivers.h"

// Stepper S-Curve Table
#define STEPPER_RAMP_STEPS 17
const uint8_t stepper_delay_table[STEPPER_RAMP_STEPS + 1] =
	{
		17, 17, 17, 16, 16, 14,
		14, 12, 11, 10, 8, 7, 7, 7, 6, 6, 6, 6};

// Global Variables
volatile char STATE = 0;			 // Kept unused
volatile uint8_t sorting_active = 0; // Non-blocking flag

// Stepper Motor globals
int current_step = 0;
int direction = 0;
volatile uint8_t stepper_position = 4;
volatile uint8_t stepper_flag = 0;
volatile uint8_t sorted_flag = 0;
volatile int step_count = 0;

volatile uint16_t stepper_steps_left = 0;
volatile int stepper_dir_request = 0;
volatile int steps_moved_so_far = 0;
volatile int same_object_flag = 0;
volatile uint16_t stepper_total_steps = 0;

// Sensor globals
volatile uint8_t OI_Counter = 0;
volatile uint8_t Entry_Flag = 0;
volatile uint8_t OR_Flag = 0;

// ADC and Reflective globals
volatile uint16_t reflective_value = 0;
volatile uint16_t ADC_result_flag = 0;
volatile uint16_t MIN_reflective_value = 1023;
volatile uint8_t OBJ_Type = 0;
volatile uint8_t OBJ_Types[100];
volatile uint8_t OBJ_Types2[100];
volatile uint8_t RETURN_TO_SCAN = 0;

// Exit, Hall Effect, and Stop globals
volatile uint8_t EX_Flag = 0;
volatile uint8_t EX_Count = 0;
element Test;
volatile uint8_t HE_Flag = 0;
volatile uint8_t Type_1 = 0;
volatile uint8_t Type_2 = 0;
volatile uint8_t Type_3 = 0;
volatile uint8_t Type_4 = 0;
volatile uint8_t Type_12 = 0;
volatile uint8_t Type_22 = 0;
volatile uint8_t Type_32 = 0;
volatile uint8_t Type_42 = 0;
volatile uint8_t stop_request_flag = 0;

// Pause globals
volatile uint8_t pause_request_flag = 0;
volatile uint8_t pause_active = 0;
volatile uint8_t saved_duty_cycle = 0;
volatile uint8_t system_paused = 0;
volatile int rampdown_flag = 0;
volatile int Total_Sorted_Count = 0;
volatile int Belt_count = 0;

volatile unsigned int timer_count = 0;

// Function Prototypes
void step(int);
void step_zero(void);
void sort(int);
void motor_stop(void);
void timerStart(void);

int main(int argc, char *argv[])
{
	CLKPR = 0x80;
	CLKPR = 0x01; // 8MHz
	TCCR1B |= _BV(CS11);
	setupTimer();

	// FIFO Setup
	link *head, *tail;
	setup(&head, &tail);
	link *newlink;
	link *deQueuedLink;
	link *temp_link;

	cli();

	// Hardware Setup
	pwmTimer();
	adc_init();
	motor_init();
	InitLCD(LS_BLINK | LS_ULINE);
	LCDClear();

	// Pin initialization
	DDRA = 0xFF;
	PORTA = 0x00;
	DDRD = 0b11110000;
	DDRC = 0xFF;

	// Pause Button
	DDRE &= ~(1 << PE4);
	PORTE |= (1 << PE4);

	// Interrupts
	EICRA |= _BV(ISC01) | _BV(ISC00);
	EICRA |= _BV(ISC11) | _BV(ISC10);
	EICRA |= _BV(ISC21);
	EICRA |= _BV(ISC31) | _BV(ISC30);
	EICRB &= ~(_BV(ISC40));
	EICRB |= _BV(ISC41);
	EIMSK |= (_BV(INT0) | _BV(INT1) | _BV(INT2) | _BV(INT3) | _BV(INT4));

	sei();

	// Startup
	motor_scurve_accel(0, 60, 400, 40);
	step_zero();

	// Main Loop
	while (1)
	{
		// 1. Stepper Driver
		if (stepper_steps_left > 0)
		{
			uint8_t delay_ms;
			uint8_t idx;

			// Checking if Accel, Decel, or cruising
			if (steps_moved_so_far < STEPPER_RAMP_STEPS)
				idx = (uint8_t)steps_moved_so_far;
			else if (stepper_steps_left <= STEPPER_RAMP_STEPS)
				idx = (uint8_t)stepper_steps_left;
			else
				idx = STEPPER_RAMP_STEPS;

			if (idx > STEPPER_RAMP_STEPS)
				idx = STEPPER_RAMP_STEPS;
			delay_ms = stepper_delay_table[idx];

			step(stepper_dir_request);
			mTimer(delay_ms);
			stepper_steps_left--;
			steps_moved_so_far++;
		}

		// 2. Pause Handling
		if (pause_request_flag && !pause_active)
		{
			pause_request_flag = 0;
			pause_active = 1;
			EIMSK &= ~_BV(INT4);
			mTimer(30);
			while (!(PINE & (1 << PE4)))
				;

			if (!system_paused)
			{
				system_paused = 1;
				saved_duty_cycle = OCR0A;
				if (saved_duty_cycle == 0)
					saved_duty_cycle = 50;
				motor_stop();
				OCR0A = 0;
				stepper_steps_left = 0;
				steps_moved_so_far = 0;
				STATE = 0;

				// Stats Display
				temp_link = head;
				while (temp_link != NULL)
				{
					temp_link = temp_link->next;
					Belt_count++;
				}

				Type_1 = 0;
				Type_2 = 0;
				Type_3 = 0;
				Type_4 = 0;
				/*Type_12 = 0; Type_22 = 0; Type_32 = 0; Type_42 = 0;*/
				for (int i = 0; i < OI_Counter; i++)
				{
					switch (OBJ_Types[i])
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
					/*switch (OBJ_Types2[i]) { case 1: Type_12++; break; case 2: Type_22++; break; case 3: Type_32++; break; case 4: Type_42++; break; }*/
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
				LCDGotoXY(0, 1);
				LCDWriteString("Belt Count: ");
				LCDWriteInt(Belt_count, 2);
				// LCDWriteString("Blt: "); LCDWriteInt(Type_12 - Type_1, 2); LCDWriteString(" "); LCDWriteInt(Type_22 - Type_2, 2); LCDWriteString(" "); LCDWriteInt(Type_32 - Type_3, 2); LCDWriteString(" "); LCDWriteInt(Type_42 - Type_4, 2);
			}
			else
			{
				Belt_count = 0;
				system_paused = 0;
				motor_apply_direction();
				OCR0A = saved_duty_cycle;
				LCDClear();
			}
			EIFR |= _BV(INTF4);
			EIMSK |= _BV(INT4);
			pause_active = 0;
		}

		// 3. Stop Request
		if (stop_request_flag == 1)
		{
			stop_request_flag = 0;
			timerStart();
			rampdown_flag = 1;
			LCDClear();
			LCDWriteString("RAMPING DOWN");
		}
		// Rampdown logic
		if (rampdown_flag == 1)
		{
			if (timer_count > 5000)
			{
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
					;
			}
		}

		// 4. Main Logic

		// A. ADC and Scanning
		if (ADC_result_flag == 1)
		{
			if (MIN_reflective_value < 250)
				OBJ_Type = 1;
			else if (MIN_reflective_value < 800)
				OBJ_Type = 2;
			else if (MIN_reflective_value < 960)
				OBJ_Type = 3;
			else
				OBJ_Type = 4;

			// for testing
			//  LCDClear();
			//  LCDWriteInt(MIN_reflective_value,4);

			initLink(&newlink);
			newlink->e.Obj_num = OI_Counter;
			newlink->e.Reflective = MIN_reflective_value;
			newlink->e.OBJ_Type = OBJ_Type;
			enqueue(&head, &tail, &newlink);

			OBJ_Types2[OI_Counter - 1] = OBJ_Type;
			stepper_flag = 1;
			ADC_result_flag = 0;
		}

		// B. Sort Trigger
		if ((EX_Flag == 1) && (sorting_active == 0))
		{
			// 1. Check if we actually have an object to sort
			if (head != NULL)
			{
				sort(head->e.OBJ_Type);
				sorting_active = 1;
				EX_Flag = 0;
				EX_Count--;
			}
			else
			{
				// 2. Empty Queue or Bounce Protection

				EX_Flag = 0;
				EX_Count--;

				if (!system_paused)
				{
					OCR0A = 60;
					motor_apply_direction();
				}
			}
		}

		// C. Sort Management
		if (sorting_active == 1)
		{
			if ((stepper_steps_left > 6) && (same_object_flag == 0))
			{
				OCR0A = 0;
			}
			else
			{
				OCR0A = 60;
				motor_apply_direction();
			}

			if (stepper_steps_left == 0)
			{
				Test = firstValue(&head);
				OBJ_Types[Test.Obj_num - 1] = Test.OBJ_Type;
				dequeue(&head, &tail, &deQueuedLink);
				free(deQueuedLink);
				Total_Sorted_Count++;

				sorted_flag = 0;
				same_object_flag = 0;
				sorting_active = 0;
			}
		}

	} // End While loop
	return 0;
}

// Local Functions

void step(int direction)
{
	if (direction == 1)
	{
		current_step++;
		if (current_step > 4)
			current_step = 1;
	}
	else
	{
		current_step--;
		if (current_step < 1)
			current_step = 4;
	}

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

// stepper zero
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

void motor_stop(void) { PORTL = 0b11110000; }

// Stepper sort
void sort(int OBJ_Type)
{
	step_count = 0;
	if (OBJ_Type == 1)
	{ // Aluminum
		switch (stepper_position)
		{
		case (1):
			stepper_position = 1;
			same_object_flag = 1;
			break;
		case (2):
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
		steps_moved_so_far = 0;
		stepper_total_steps = step_count;
	}
}

// Background timer start (for rampdown)
void timerStart()
{
	timer_count = 0;
	TCNT3 = 0;
}

// ISRs

// Optical Sensor
ISR(INT0_vect)
{
	Entry_Flag = 1;
	reflective_value = 0;
	MIN_reflective_value = 1023;
	OI_Counter++;
	ADCSRA |= _BV(ADSC);
}

// EX Sensor
ISR(INT1_vect)
{
	EX_Flag = 1;
	EX_Count++;
	motor_stop();
	OCR0A = 0;
}

// HE Sensor
ISR(INT2_vect) { HE_Flag = 1; }

// Ramdown Button ISR
ISR(INT3_vect) { stop_request_flag = 1; }

// Pause Button ISR
ISR(INT4_vect)
{
	pause_request_flag = 1;
	EIFR |= _BV(INTF4);
}

// ADC conversions
ISR(ADC_vect)
{
	reflective_value = ADC;
	if (reflective_value < MIN_reflective_value)
	{
		MIN_reflective_value = reflective_value;
	}

	if (OR_SENSOR_PORT & (1 << OR_SENSOR_PIN))
	{
		// Object is present
		ADCSRA |= _BV(ADSC); // Keep scanning
	}
	else
	{
		// Object appears gone.
		// Only finalize if belt is moving. (Fixes motor stop glitch)
		ADC_result_flag = 1;
	}
}

// BAD ISR
ISR(BADISR_vect) {}

// Background timer ISR, for Rampdown
ISR(TIMER3_COMPA_vect)
{
	timer_count++;
}
