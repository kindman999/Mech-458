#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "lcd.h"
#include "LinkedQueue.h"
#include "drivers.h"

// --- STEPPER S-CURVE TABLE ---
#define STEPPER_RAMP_STEPS 17
const uint8_t stepper_delay_table[STEPPER_RAMP_STEPS + 1] =
	{
		17, 17, 17, 17, 16, 16, 16, 16,
		15, 14, 12, 11, 10, 8, 7, 6, 6, 6};

// --- GLOBAL VARIABLES ---
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

// ADC / Reflective globals
volatile uint16_t reflective_value = 0;
volatile uint16_t ADC_result_flag = 0;
volatile uint16_t MIN_reflective_value = 1023; // Original name
volatile uint8_t OBJ_Type = 0;
volatile uint8_t OBJ_Types[100];
volatile uint8_t OBJ_Types2[100];
volatile uint8_t RETURN_TO_SCAN = 0;

// Exit / HE / Stop globals
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

// PAUSE GLOBALS
volatile uint8_t pause_request_flag = 0;
volatile uint8_t pause_active = 0;
volatile uint8_t saved_duty_cycle = 0;
volatile uint8_t system_paused = 0;
volatile int rampdown_flag = 0;
volatile int Total_Sorted_Count = 0;

// --- FUNCTION PROTOTYPES ---
void step(int);
void step_zero(void);
void sort(int);
void motor_stop(void);

int main(int argc, char *argv[])
{
	CLKPR = 0x80;
	CLKPR = 0x01; // 8MHz
	TCCR1B |= _BV(CS11);

	// FIFO Setup
	link *head, *tail;
	setup(&head, &tail);
	link *newlink;
	link *deQueuedLink;

	cli();

	// Hardware Setup
	pwmTimer();
	adc_init();
	motor_init();
	InitLCD(LS_BLINK | LS_ULINE);
	LCDClear();

	// Pins
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
	motor_scurve_accel(0, 80, 400, 40);
	step_zero();

	// --- MAIN LOOP ---
	while (1)
	{
		// 1. STEPPER DRIVER
		if (stepper_steps_left > 0)
		{
			uint8_t delay_ms;
			uint8_t idx;

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

		// 2. PAUSE HANDLING
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
				Type_1 = 0;
				Type_2 = 0;
				Type_3 = 0;
				Type_4 = 0;
				Type_12 = 0;
				Type_22 = 0;
				Type_32 = 0;
				Type_42 = 0;
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
					switch (OBJ_Types2[i])
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
				LCDWriteString("Blt: ");
				LCDWriteInt(Type_12 - Type_1, 2);
				LCDWriteString(" ");
				LCDWriteInt(Type_22 - Type_2, 2);
				LCDWriteString(" ");
				LCDWriteInt(Type_32 - Type_3, 2);
				LCDWriteString(" ");
				LCDWriteInt(Type_42 - Type_4, 2);
			}
			else
			{
				system_paused = 0;
				motor_apply_direction();
				OCR0A = saved_duty_cycle;
				LCDClear();
			}
			EIFR |= _BV(INTF4);
			EIMSK |= _BV(INT4);
			pause_active = 0;
		}

		// 3. STOP REQUEST
		if (stop_request_flag)
		{
			stop_request_flag = 0;
			rampdown_flag = 1;
			LCDClear();
			LCDWriteString("RAMPING DOWN");
		}

		if (rampdown_flag == 1)
		{
			if ((OI_Counter == Total_Sorted_Count) && (stepper_steps_left == 0))
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

		// -------------------------------------------------------------
		// 4. MAIN LOGIC
		// -------------------------------------------------------------

		// A. ADC / SCANNING
		if (ADC_result_flag == 1)
		{
			if (MIN_reflective_value < 250)
				OBJ_Type = 1;
			else if (MIN_reflective_value < 750)
				OBJ_Type = 2;
			else if (MIN_reflective_value < 960)
				OBJ_Type = 3;
			else
				OBJ_Type = 4;

			initLink(&newlink);
			newlink->e.Obj_num = OI_Counter;
			newlink->e.Reflective = MIN_reflective_value;
			newlink->e.OBJ_Type = OBJ_Type;
			enqueue(&head, &tail, &newlink);

			OBJ_Types2[OI_Counter - 1] = OBJ_Type;
			stepper_flag = 1;
			ADC_result_flag = 0;
		}

		// B. SORT TRIGGER
		if ((EX_Flag == 1) && (sorting_active == 0))
		{
			if (head != NULL)
			{
				sort(head->e.OBJ_Type);
				sorting_active = 1;
				EX_Flag = 0;
				EX_Count--;
			}
			else
			{
				EX_Flag = 0;
				EX_Count--;
			}
		}

		// C. SORT MANAGEMENT
		if (sorting_active == 1)
		{
			if ((stepper_steps_left > 8) && (same_object_flag == 0))
			{
				OCR0A = 0; // Stop Belt
			}
			else
			{
				if (OCR0A == 0 && !system_paused)
				{
					OCR0A = 80;
					motor_apply_direction();
				}

				if (stepper_steps_left == 0)
				{
					if (head != NULL)
					{
						Test = firstValue(&head);
						OBJ_Types[Test.Obj_num - 1] = Test.OBJ_Type;
						dequeue(&head, &tail, &deQueuedLink);
						free(deQueuedLink);
						Total_Sorted_Count++;
					}
					sorted_flag = 0;
					same_object_flag = 0;
					sorting_active = 0;
				}
			}
		}

	} // End While(1)
	return 0;
}

// --- LOCAL FUNCTIONS ---

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

ISR(INT2_vect) { HE_Flag = 1; }

ISR(INT3_vect) { stop_request_flag = 1; }

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
		// Object is present
		ADCSRA |= _BV(ADSC); // Keep scanning
	}
	else
	{
		// Object appears gone.
		// ONLY finalize if belt is moving. (Fixes motor stop glitch)
		if (OCR0A > 0)
		{
			ADC_result_flag = 1;
		}
		else
		{
			// Belt stopped? Likely noise. Keep checking until belt restarts.
			ADCSRA |= _BV(ADSC);
		}
	}
}

ISR(BADISR_vect) {}