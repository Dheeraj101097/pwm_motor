#include <avr/io.h>
#include <avr/interrupt.h>
#define TEMP_THRESHOLD 30					// Temperature threshold in °C

void adc_init() {
    ADMUX = (1 << REFS0); 			     
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 
}

uint16_t read_adc() { 
    ADCSRA |= (1 << ADSC); 							// Start ADC conversion 
    while (ADCSRA & (1 << ADSC)); 				        // Wait for conversion to complete
    return ADC;
}

void pwm_init() { 
    DDRD |= (1 << PD6); 						    // Set PD6 (OC0A) as output 
    TCCR0A = (1 << WGM01) | (1 << WGM00) | (1 << COM0A1);      // Fast PWM mode, non-inverting 
    TCCR0B = (1 << CS01) | (1 << CS00);					        // Prescaling to 64
}

void set_pwm_duty(uint8_t duty) {
    OCR0A = duty; 								  // Set PWM duty cycle
}

void timer1_init() { 
    TCCR1B |= (1 << WGM12); 							      // CTC mode
    OCR1A = 15624;   			 // Set compare value for 1-second delay (1024 prescaler) 
    TIMSK1 |= (1 << OCIE1A); 			// Enable Timer1 Compare A Match Interrupt
    TCCR1B |= (1 << CS12) | (1 << CS10);    		   // Start Timer1 with 1024 prescaler
}

ISR(TIMER1_COMPA_vect) {
    uint16_t adc_value = read_adc(); 			        // Read temperature from LM35
    uint16_t temperature = (adc_value * 4.88) / 10; // Convert ADC value to temperature in °C
    if (temperature >= TEMP_THRESHOLD) {
        set_pwm_duty(0); 				 // Turn off motor if temperature is too high
    } else {
        uint8_t duty = (temperature * 2); 		     // Map temperature to PWM (e.g., 0-255)
        set_pwm_duty(duty);
    }
}
int main() {
    adc_init(); 										// Initialize ADC
    pwm_init(); 							    // Initialize PWM on Timer0
    timer1_init(); 					     // Initialize Timer1 for 1-second interrupt
    sei(); 								       // Enable global interrupts
    while (1) { }
}

