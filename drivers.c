#include "drivers.h"

uint8_t motor_direction_cw = 1;

// mTimer, used for stepper delay
void mTimer(int count)
{
	int i = 0;
	TCCR1B |= _BV(WGM12);
	OCR1A = 0x03E8; // 1 ms
	TCNT1 = 0x0000;
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

// Background timer (rampdowwn)
void setupTimer()
{

	cli();

	TCCR3B |= _BV(WGM12); // clear on compare match
	OCR3A = 124;
	TCCR3B |= _BV(CS31) | _BV(CS30); // 64 prescaler
	TIMSK3 |= _BV(OCIE3A);

	sei();
}

// PWM
void pwmTimer()
{
	// Set Timer0 to Fast PWM mode (WGM00 + WGM01 = 1)
	TCCR0A |= (1 << WGM00) | (1 << WGM01);
	TCCR0B &= ~(1 << WGM02); // WGM02 = 0

	// Set non-inverting mode on OC0A
	TCCR0A |= (1 << COM0A1);
	TCCR0A &= ~(1 << COM0A0);

	// Set prescaler (clk/64)
	TCCR0B |= (1 << CS01) | (1 << CS00);

	OCR0A = 0;
	DDRB |= (1 << PB7); // Set PB7 as output
}

// ADC
void adc_init(void)
{
	// Vref = AVcc, right adjust result, channel ADC0
	ADMUX = _BV(REFS0);

	// Free-running disabled; retrigger in ISR
	ADCSRA = _BV(ADEN) | _BV(ADIE); // enable ADC + interrupt
	ADCSRB = 0x00;

	// Prescaler /32
	ADCSRA |= _BV(ADPS2) | _BV(ADPS0);
}

// Initialize DC Motor pins
void motor_init(void)
{
	// Direction pins
	MOTOR_DIR_DDR |= MOTOR_PIN_IN1 | MOTOR_PIN_IN2;
	// Enable pins
	MOTOR_ENA_DDR |= MOTOR_ENA_PIN_A | MOTOR_ENA_PIN_B;
	// Enable EA/EB
	MOTOR_ENA_PORT |= MOTOR_ENA_PIN_A | MOTOR_ENA_PIN_B;

	// Default direction CW
	motor_apply_direction();
}

// Apply DC Motor Direction
void motor_apply_direction(void)
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

// Change DC Duty Cycle
void motor_set_speed(uint8_t duty)
{
	OCR0A = duty;
}

// Jerk-limited S-curve acceleration
void motor_scurve_accel(uint8_t start_duty, uint8_t target_duty, uint16_t duration_ms, uint8_t steps)
{
	if (steps == 0 || duration_ms == 0)
	{
		motor_set_speed(target_duty);
		return;
	}

	int16_t delta = (int16_t)target_duty - (int16_t)start_duty;
	uint16_t dt = duration_ms / steps;
	if (dt == 0)
		dt = 1;

	motor_set_speed(start_duty);

	for (uint8_t i = 0; i <= steps; i++)
	{
		float t = (float)i / (float)steps;
		// Smoothstep S-curve profile: s(t) = 3 t^2 - 2 t^3
		float s = t * t * (3.0f - 2.0f * t);
		float duty_f = (float)start_duty + (float)delta * s;

		if (duty_f < 0.0f)
			duty_f = 0.0f;
		else if (duty_f > 255.0f)
			duty_f = 255.0f;

		motor_set_speed((uint8_t)(duty_f + 0.5f));
		mTimer(dt);
	}
}

void motor_scurve_decel(uint8_t start_duty, uint8_t target_duty, uint16_t duration_ms, uint8_t steps)
{
	motor_scurve_accel(start_duty, target_duty, duration_ms, steps);
}