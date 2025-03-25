#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>

// --- Define constants for pin assignments ---
#define HOUR_LED_BIT0  PD0
#define HOUR_LED_BIT1  PD7
#define HOUR_LED_BIT2  PD4
#define HOUR_LED_BIT3  PD5
#define HOUR_LED_BIT4  PD6

#define MIN_BUTTON      PD1
#define HOUR_BUTTON     PD2 // INT0 (PD2)
#define BRIGHTNESS_BUTTON PD3 // INT1 (PD3)

// --- Global variables ---
volatile uint8_t seconds = 0;
volatile uint8_t minutes = 20;
volatile uint8_t hours = 5;
volatile uint8_t brightness_stage = 0; 	   // Tracks the current brightness stage (0-3)
//volatile uint8_t pwm_hold_timer = 0;       // Tracks how long the brightness button is held
//volatile uint8_t pwm_button_held = 0;      // Flag indicating if the brightness button is currently held
volatile uint8_t min_hold_timer = 0;	   // Tracks how long the minute button is held
volatile uint8_t min_button_held = 0;	   // Flag indicating if the minute button is currently held
//volatile uint8_t hour_hold_timer = 0;	   // Tracks how long the hour button is held
//volatile uint8_t hour_button_held = 0;	   // Flag indicating if the hour button is currently held

const uint8_t brightness_levels[] = {255, 248, 156, 0}; // Define brightness levels (in reversed order)

// --- Function to set up Timer2 ---
void setup_timer2() {
    // Activate asynchronous mode for Timer2 with external clock
    ASSR |= (1 << AS2);

    // Set Timer2 to Normal mode
    TCCR2A = 0;                     	    // Normal mode
    TCCR2B = (1 << CS22) | (1 << CS20);     // Prescaler = 128

    // Enable Timer2 Overflow interrupt
    TIMSK2 = (1 << TOIE2);

    // Wait for asynchronous clock stabilization
    while (ASSR & ((1 << TCR2AUB) | (1 << TCR2BUB))) {
        // Wait until Timer2 registers are updated
    }
}

// --- Function to set up Timer1 ---
void setup_timer1() {
    // Configure Timer1 for PWM on PB1 and PB2
    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1); // 8-bit Fast PWM, Clear on Compare Match
    TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler = 8

    // Set initial brightness for Hours and Minutes LEDs
    OCR1A = brightness_levels[brightness_stage];
    OCR1B = brightness_levels[brightness_stage];
}

// --- Function to set up ports ---
void setup_ports() {
    // Configure hour LEDs on PD7, PD0, PD4, PD5, PD6 as outputs
    DDRD |= (1 << HOUR_LED_BIT1) | (1 << HOUR_LED_BIT0) | (1 << HOUR_LED_BIT2) | (1 << HOUR_LED_BIT3) | (1 << HOUR_LED_BIT4);
    
    // Configure minute LEDs on PC0, PC1, PC2, PC3, PC4, PC5 as outputs
    DDRC |= 0x3F;  // PC0 to PC5 as outputs
    
    // Configure PB1 and PB2 for PWM cathode control as outputs
    DDRB |= (1 << PB1) | (1 << PB2);

    // Disable ADC, USART, SPI, Timer0, and TWI to save power
    PRR |= (1 << PRADC) | (1 << PRUSART0) | (1 << PRSPI) | (1 << PRTIM0) | (1 << PRTWI);

    // Set all other PORTB pins as inputs with pull-up resistors (except PB1, PB2)
    DDRB &= ~((1 << PB0) | (1 << PB3) | (1 << PB4) | (1 << PB5) | (1 << PB6) | (1 << PB7));
    PORTB |=  ((1 << PB0) | (1 << PB3) | (1 << PB4) | (1 << PB5) | (1 << PB6) | (1 << PB7));

    // Configure buttons on PD1, PD2, PD3 as inputs with pull-up resistors
    DDRD &= ~((1 << MIN_BUTTON) | (1 << HOUR_BUTTON) | (1 << BRIGHTNESS_BUTTON));
    PORTD |= (1 << MIN_BUTTON) | (1 << HOUR_BUTTON) | (1 << BRIGHTNESS_BUTTON);

    // Turn off LEDs initially (anodes low, cathodes off via PWM)
    PORTD &= ~((1 << HOUR_LED_BIT1) | (1 << HOUR_LED_BIT0) | (1 << HOUR_LED_BIT2) | (1 << HOUR_LED_BIT3) | (1 << HOUR_LED_BIT4));
    PORTC &= ~0x3F;
    OCR1A = brightness_levels[brightness_stage];
    OCR1B = brightness_levels[brightness_stage];
}

// --- Function to set up interrupts ---
void setup_interrupts() {
    // Configure external interrupts INT0 and INT1
    EICRA |= (1 << ISC01); // INT0: Falling edge (HOUR_BUTTON - PD2)
    EICRA &= ~(1 << ISC00);
    EICRA |= (1 << ISC11); // INT1: Falling edge (BRIGHTNESS_BUTTON - PD3)
    EICRA &= ~(1 << ISC10);

    // Enable INT0 and INT1
    EIMSK |= (1 << INT0) | (1 << INT1);

    // Enable Pin Change Interrupt for MIN_BUTTON (PD1)
    PCICR |= (1 << PCIE2); // Enable Pin Change Interrupt for PORTD
    PCMSK2 |= (1 << PCINT17); // Enable interrupt for PD1 (MIN_BUTTON)
}

// --- Function to update the LED display ---
void update_display() {
    // Clear hour LEDs
    PORTD &= ~((1 << HOUR_LED_BIT1) | (1 << HOUR_LED_BIT0) | (1 << HOUR_LED_BIT2) | (1 << HOUR_LED_BIT3) | (1 << HOUR_LED_BIT4));
    
    // Set hour LEDs based on the current hour value
    if (hours & (1 << 0)) PORTD |= (1 << HOUR_LED_BIT0); // Hour 1 (PD0)
    if (hours & (1 << 1)) PORTD |= (1 << HOUR_LED_BIT1); // Hour 2 (PD7)
    if (hours & (1 << 2)) PORTD |= (1 << HOUR_LED_BIT2); // Hour 4 (PD4)
    if (hours & (1 << 3)) PORTD |= (1 << HOUR_LED_BIT3); // Hour 8 (PD5)
    if (hours & (1 << 4)) PORTD |= (1 << HOUR_LED_BIT4); // Hour 16 (PD6)

    // Clear minute LEDs
    PORTC &= ~0x3F;
    
    // Set minute LEDs based on the current minute value
    PORTC |= (minutes & 0x3F); // Minutes directly on PC0-PC5
}

// --- Timer2 Overflow Interrupt Service Routine ---
ISR(TIMER2_OVF_vect) {
    // Increment seconds
    seconds++;

    // Handle minute button hold timing
    if (min_button_held) {
        min_hold_timer++;
        if (min_hold_timer >= 1) { // ~1 second hold
            minutes--;
            seconds = 0;
            min_button_held = 0; // Stop further timing
        }
    }

    // Increment minutes and hours as needed
    if (seconds >= 60) {
        seconds = 0;
        minutes++;
        if (minutes >= 60) {
            minutes = 0;
            hours++;
            if (hours >= 24) {
                hours = 0; // Reset after 24 hours
            }
        }
    }

    // Update the LED display
    update_display();
}

// --- External Interrupt 0 Service Routine (HOUR_BUTTON) ---
ISR(INT0_vect) {
    // Debounce the button press
    _delay_ms(5);
    if (!(PIND & (1 << HOUR_BUTTON))) { // Confirm button is still pressed
        hours++;
        if (hours >= 24) {
            hours = 0;
        }
        update_display();
    }
}

// --- External Interrupt 1 Service Routine (BRIGHTNESS_BUTTON) ---
ISR(INT1_vect) {
    // Debounce the button press
    _delay_ms(5);
    if (!(PIND & (1 << BRIGHTNESS_BUTTON))) { // Confirm button is still pressed
        brightness_stage = (brightness_stage + 1) % 4;
        OCR1A = brightness_levels[brightness_stage];
        OCR1B = brightness_levels[brightness_stage];
    }
}

// --- Pin Change Interrupt 2 Service Routine (MIN_BUTTON) ---
ISR(PCINT2_vect) {
    // Handle minute button press and release
    if (!(PIND & (1 << MIN_BUTTON))) { // Button pressed
        _delay_ms(5); // Debounce
        if (!(PIND & (1 << MIN_BUTTON))) { // Confirm button is still pressed
            min_button_held = 1; // Start hold timing
            min_hold_timer = 0;  // Reset timer
        }
    } else { // Button released
        _delay_ms(5); // Debounce
        if ((PIND & (1 << MIN_BUTTON))) { // Confirm button is released
            if (min_button_held && min_hold_timer < 1) { // Short press
                minutes++;
                seconds = 0;
                if (minutes >= 60) {
                    minutes = 0;
                    hours++;
                    if (hours >= 24) {
                        hours = 0;
                    }
                }
            }
            min_button_held = 0; // Reset hold flag
            update_display(); // Update display immediately
        }
    }
}

// --- Function to enter sleep mode ---
void enter_sleep() {
    if (OCR1A == 255 && OCR1B == 255) { // Check if LEDs are off (PWM duty cycle 255)
        set_sleep_mode(SLEEP_MODE_PWR_SAVE); // Power-Down mode if LEDs are off
        sleep_enable();                      // Enable sleep
        sleep_cpu();                         // Enter sleep mode
    } else {
        set_sleep_mode(SLEEP_MODE_IDLE); // Idle mode if LEDs are on (for PWM)
        sleep_enable();                      // Enable sleep
        sleep_cpu();                         // Enter sleep mode
    }
    // CPU wakes up here after interrupt
    sleep_disable(); // Disable sleep after waking up
}

// --- Main function ---
int main() {
    setup_ports();        // Configure LED and button pins
    setup_timer2();       // Configure Timer2 for timekeeping
    setup_timer1();       // Configure Timer1 for PWM on PB1 and PB2
    setup_interrupts();   // Configure interrupts for buttons

    sei();  // Enable global interrupts

    brightness_stage = 2; // Set initial brightness to medium
    OCR1A = brightness_levels[brightness_stage]; // Set initial brightness level
    OCR1B = brightness_levels[brightness_stage]; // Set initial brightness level
    update_display(); // Update the display initially

    while (1) {
        asm("nop"); // No operation (placeholder)
        enter_sleep(); // Enter sleep mode and wait for interrupt
        _delay_ms(5); 
    }

    return 0;
}
