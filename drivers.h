#ifndef DRIVERS_H
#define DRIVERS_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

// --- PIN DEFINITIONS ---
// DC Motor
#define MOTOR_DIR_PORT PORTL
#define MOTOR_DIR_DDR  DDRL
#define MOTOR_PIN_IN1  (1 << PL7) // motor input A
#define MOTOR_PIN_IN2  (1 << PL6) // motor input B
#define MOTOR_ENA_PORT PORTL
#define MOTOR_ENA_DDR  DDRL
#define MOTOR_ENA_PIN_A (1 << PL4) // EA
#define MOTOR_ENA_PIN_B (1 << PL5) // EB

// Sensors
#define OR_SENSOR_PIN PD0
#define OR_SENSOR_PORT PIND

// --- FUNCTION PROTOTYPES ---
// Hardware drivers implemented in drivers.c
void adc_init(void);
void pwmTimer(void);
void mTimer(int count);
void motor_init(void);
void motor_set_speed(uint8_t duty);
void motor_scurve_accel(uint8_t start_duty, uint8_t target_duty, uint16_t duration_ms, uint8_t steps);
void motor_scurve_decel(uint8_t start_duty, uint8_t target_duty, uint16_t duration_ms, uint8_t steps);
void motor_apply_direction(void);

// Extern global for motor direction (needed by drivers.c)
extern uint8_t motor_direction_cw;

#endif