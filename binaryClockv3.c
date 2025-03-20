#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>

// --- Konstanten für Pins definieren ---
#define HOUR_LED_BIT0  PD0
#define HOUR_LED_BIT1  PD7
#define HOUR_LED_BIT2  PD4
#define HOUR_LED_BIT3  PD5
#define HOUR_LED_BIT4  PD6

#define MINUTE_LED_PORT PORTC // Ganzer Port für Minuten
#define HOUR_LED_PORT   PORTD // Ganzer Port für Stunden (teilweise)

#define MIN_BUTTON      PD1
#define HOUR_BUTTON     PD2 // INT0 (PD2)
#define BRIGHTNESS_BUTTON PD3 // INT1 (PD3)

// --- Hold Zeiten Konstanten ---
#define HOLD_TIME_MS 1000 // Hold time in milliseconds (e.g., 1000ms = 1 second)
#define CHECK_INTERVAL_MS 50 // Interval to check button hold status in Timer0 (e.g., 50ms)
#define HOLD_COUNT_THRESHOLD (HOLD_TIME_MS / CHECK_INTERVAL_MS) // Number of Timer0 interrupts for hold

volatile uint8_t seconds = 0;
volatile uint8_t minutes = 20;
volatile uint8_t hours = 5;
volatile uint8_t brightness_stage = 0;
volatile uint16_t pwm_hold_counter = 0; // Counter for brightness button hold time
volatile uint8_t pwm_button_held = 0;
volatile uint16_t hour_hold_counter = 0; // Counter for hour button hold time
volatile uint8_t hour_button_held = 0;
volatile uint8_t min_hold_timer = 0;
volatile uint8_t min_button_held = 0;


const uint8_t brightness_levels[] = {255, 248, 156, 0}; // Define brightness levels (in reversed order)


void setup_timer2() {
    // Activate asynchronous mode for Timer2 with external clock (instead of synchronus) - Keine externe Clock laut Schematic
    ASSR |= (1 << AS2);

    // Set Timer2 to Normal mode (instead of CTC or PWM)
    TCCR2A = 0;                     	    // Normal mode
    TCCR2B = (1 << CS22) | (1 << CS20);     // Prescaler = 128

    // Enable Timer2 Overflow interrupt
    TIMSK2 = (1 << TOIE2);

    // Wait for asynchronous clock stabilization
    while (ASSR & ((1 << TCR2AUB) | (1 << TCR2BUB))) { // Keep wait loop, might be important for internal clock stabilization
        // Wait until Timer2 registers are updated
    }
}

void setup_timer1() {
    // Configure Timer1 for PWM on PB1 and PB2
    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1); // 8-bit Fast PWM, Clear on Compare Match
    TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler = 8

    OCR1A = brightness_levels[brightness_stage]; // Set initial brightness for Hours LEDs
    OCR1B = brightness_levels[brightness_stage]; // Set initial brightness for Minutes LEDs
}

void setup_timer0_hold_check() {
    // Timer0 for checking button hold status (CTC mode)
    TCCR0A = (1 << WGM01);        // CTC Mode
    TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64 (for ~1ms ticks at 8MHz) - ADJUST PRESCALER IF NEEDED FOR 50ms INTERVAL
    OCR0A = 124;                  // Compare Value for ~1ms interrupt (8MHz / 64 / 1000 = 125 -1) - ADJUST OCR0A TO GET CHECK_INTERVAL_MS
    TIMSK0 |= (1 << OCIE0A);       // Compare Match A Interrupt aktivieren
}


void setup_ports() {
    // Stunden LEDs an PD7, PD0, PD4, PD5, PD6 (Anoden) und PB1 (Kathoden/PWM)
    DDRD |= (1 << PD7) | (1 << PD0) | (1 << PD4) | (1 << PD5) | (1 << PD6); // Stunden LEDs als Ausgänge
    // Minuten LEDs an PC0, PC1, PC2, PC3, PC4, PC5 (Anoden) und PB2 (Kathoden/PWM)
    DDRC |= 0x3F;  // PC0 bis PC5 für Minuten LEDs als Ausgänge
    // Kathodensteuerung über PB1 und PB2 (PWM)
    DDRB |= (1 << PB1) | (1 << PB2);      // PB1 und PB2 als Ausgänge für PWM Kathodensteuerung

    // Deaktiviere ADC, USART, SPI, Timer und TWI
    PRR |= (1 << PRADC) | (1 << PRUSART0) | (1 << PRSPI) | (1 << PRTIM0) | (1 << PRTWI);

    // Alle anderen Pins von PORTB als Eingang mit Pull-Up (ausser PB1, PB2)
    DDRB &= ~((1 << PB0) | (1 << PB3) | (1 << PB4) | (1 << PB5) | (1 << PB6) | (1 << PB7));
    PORTB |=  ((1 << PB0) | (1 << PB3) | (1 << PB4) | (1 << PB5) | (1 << PB6) | (1 << PB7));


    // Buttons an PD1, PD2, PD3 als Eingänge mit Pull-Up
    DDRD &= ~((1 << PD1) | (1 << PD2) | (1 << PD3)); // PD1, PD2, PD3 als Eingänge
    PORTD |= (1 << PD1) | (1 << PD2) | (1 << PD3);  // Pull-Ups für Buttons aktivieren

    // LEDs initial ausschalten (Anoden auf Low, Kathoden über PWM aus)
    PORTD &= ~((1 << PD7) | (1 << PD0) | (1 << PD4) | (1 << PD5) | (1 << PD6)); // Stunden LEDs aus
    PORTC &= ~0x3F; // Minuten LEDs aus
    OCR1A = brightness_levels[brightness_stage]; // PWM für Stunden LEDs initial brightness level
    OCR1B = brightness_levels[brightness_stage]; // PWM für Minuten LEDs initial brightness level
}


void setup_interrupts() {
    // --- External Interrupts INT0 und INT1 konfigurieren ---
    // EICRA Register: Trigger-Typ für INT0 und INT1 setzen
    EICRA |= (1 << ISC01); // INT0: Fallende Flanke (ISC01=1, ISC00=0) - HOUR_BUTTON (PD2)
    EICRA &= ~(1 << ISC00);
    EICRA |= (1 << ISC11); // INT1: Fallende Flanke (ISC11=1, ISC10=0) - BRIGHTNESS_BUTTON (PD3)
    EICRA &= ~(1 << ISC10);

    // EIMSK Register: INT0 und INT1 aktivieren
    EIMSK |= (1 << INT0) | (1 << INT1);

    // --- Pin Change Interrupt für MIN_BUTTON (PD1) beibehalten und für Button Release Detection für INT0 und INT1 ---
    PCICR |= (1 << PCIE2); // Pin-Change Interrupt für PORTD aktivieren
    PCMSK2 |= (1 << PCINT17) | (1 << PCINT18) | (1 << PCINT19); // PD1, PD2 und PD3 für Release Detection
}

void update_display() {
    // Stunden LEDs: PD7, PD0, PD4, PD5, PD6
    // Minuten LEDs: PC0, PC1, PC2, PC3, PC4, PC5

    // Stunden LEDs setzen
    PORTD &= ~((1 << PD7) | (1 << PD0) | (1 << PD4) | (1 << PD5) | (1 << PD6)); // Stunden LEDs zuerst löschen
    if (hours & (1 << 0)) PORTD |= (1 << PD0); // Stunde 1 (PD0)
    if (hours & (1 << 1)) PORTD |= (1 << PD7); // Stunde 2 (PD7)
    if (hours & (1 << 2)) PORTD |= (1 << PD4); // Stunde 4 (PD4)
    if (hours & (1 << 3)) PORTD |= (1 << PD5); // Stunde 8 (PD5)
    if (hours & (1 << 4)) PORTD |= (1 << PD6); // Stunde 16 (PD6)


    // Minuten LEDs setzen
    PORTC &= ~0x3F; // Minuten LEDs zuerst löschen
    MINUTE_LED_PORT |= (minutes & 0x3F); // Minuten direkt auf PC0-PC5 schreiben, da sie sequenziell sind.
}


ISR(TIMER2_OVF_vect) {
    // Increment seconds
    seconds++;

    if (seconds >= 60) {
        seconds = 0;
        minutes++;
        if (minutes >= 60) {
            minutes = 0;
            hours++;
            if (hours >= 24) {
                hours = 0; // Reset after 24 Stunden
            }
        }
    }

    update_display(); // LED Anzeige aktualisieren
}

ISR(TIMER0_COMPA_vect) {
    // Timer0 Compare Match ISR - Check for button hold status

    // Check Brightness Button Hold
    if (pwm_button_held) {
        pwm_hold_counter++;
        if (pwm_hold_counter >= HOLD_COUNT_THRESHOLD) {
            pwm_hold_counter = 0; // Reset counter
            // Perform brightness DOWN action (subtract/decrement brightness) on hold
            if (brightness_stage > 0) {
                brightness_stage--;
            } else {
                brightness_stage = 3; // Cycle to lowest brightness if already at 0
            }
            OCR1A = brightness_levels[brightness_stage];
            OCR1B = brightness_levels[brightness_stage];
        }
    }

    // Check Hour Button Hold
    if (hour_button_held) {
        hour_hold_counter++;
        if (hour_hold_counter >= HOLD_COUNT_THRESHOLD) {
            hour_hold_counter = 0; // Reset counter
            // Perform hour DOWN action (subtract/decrement hours) on hold
            if (hours > 0) {
                hours--;
            } else {
                hours = 23; // Cycle to 23 if already at 0
            }
            update_display();
        }
    }
}


ISR(INT0_vect) {
    // ISR für INT0 (HOUR_BUTTON - PD2)
    // Handle HOUR_BUTTON press (START HOLD DETECTION TIMER, PERFORM SHORT PRESS ACTION)
    _delay_ms(20); // Debounce
    if (!(PIND & (1 << HOUR_BUTTON))) { // Bestätige Button ist immer noch gedrückt
        hour_button_held = 1; // Set hour button held flag - START HOLD TIMER IN TIMER0 ISR
        hour_hold_counter = 0;  // Reset hour hold counter

        // Short press action (add/increment hours)
        hours++;
        if (hours >= 24) {
            hours = 0;
        }
        update_display();
    }
}

ISR(INT1_vect) {
    // ISR für INT1 (BRIGHTNESS_BUTTON - PD3)
    // Handle BRIGHTNESS_BUTTON press (START HOLD DETECTION TIMER, PERFORM SHORT PRESS ACTION)
    _delay_ms(20); // Debounce
    if (!(PIND & (1 << BRIGHTNESS_BUTTON))) { // Bestätige Button ist immer noch gedrückt
        pwm_button_held = 1; // Set brightness button held flag - START HOLD TIMER IN TIMER0 ISR
        pwm_hold_counter = 0;  // Reset brightness hold counter

        // Short press action (cycle brightness up)
        brightness_stage = (brightness_stage + 1) % 4; // Cycle brightness up for short press
        OCR1A = brightness_levels[brightness_stage];
        OCR1B = brightness_levels[brightness_stage];
    }
}


ISR(PCINT2_vect) {
    // ISR für PCINT2 (wird jetzt für MIN_BUTTON - PD1 verwendet UND BUTTON RELEASE DETECTION für ALLE BUTTONS)
    // Handle PD1 button (Increment minutes) - SW1
    if ( !(PIND & (1 << PD1))) { // Button gedrückt
        _delay_ms(5); // Debounce
        if (!(PIND & (1 << PD1))) { // Bestätige Button ist immer noch gedrückt
            min_button_held = 1; // Hold Timing starten
            min_hold_timer = 0;  // Timer zurücksetzen
        }
    } else { // Button released - **RELEASE DETECTION FOR ALL BUTTONS HERE**
        _delay_ms(5); // Debounce
        if ((PIND & (1 << PD1))) { // MIN_BUTTON Released
	        if (min_button_held && min_hold_timer < 1) { // Normaler kurzer Tastendruck
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
	        min_button_held = 0; // Reset Hold Flag
            update_display(); // Sofort updaten
        }
        if ((PIND & (1 << BRIGHTNESS_BUTTON))) { // BRIGHTNESS_BUTTON Released
            pwm_button_held = 0; // **RESET pwm_button_held FLAG ON RELEASE!** - STOP HOLD TIMER IN TIMER0 ISR
            pwm_hold_counter = 0; // Reset hold counter for robustness
        }
        if ((PIND & (1 << HOUR_BUTTON))) { // HOUR_BUTTON Released
            hour_button_held = 0; // **RESET hour_button_held FLAG ON RELEASE!** - STOP HOLD TIMER IN TIMER0 ISR
            hour_hold_counter = 0; // Reset hold counter for robustness
        }
    }
}


void enter_sleep() {
    if(OCR1A == 0 && OCR1B == 0) { // Überprüfe ob LEDs aus sind (PWM duty cycle 0 - Corrected condition)
        set_sleep_mode(SLEEP_MODE_PWR_SAVE); // Power-Down Modus wenn LEDs aus
        sleep_enable();                      // Sleep aktivieren
        sleep_cpu();                         // CPU schlafen legen
    } else {
	    set_sleep_mode(SLEEP_MODE_IDLE); // Idle Modus wenn LEDs an (für PWM)
        sleep_enable();                      // Sleep aktivieren
        sleep_cpu();                         // CPU schlafen legen
    }
    // CPU wacht hier nach Interrupt auf
    sleep_disable(); // Sleep deaktivieren nach Aufwachen
}


int main() {
    setup_ports();        // LED und Button Pins konfigurieren
    setup_timer2();       // Timer2 für Zeitmessung konfigurieren
    setup_timer1();       // Timer1 für PWM auf PB1 und PB2 konfigurieren
    setup_timer0_hold_check(); // Timer0 for hold check
    setup_interrupts();   // Interrupts für Buttons konfigurieren

    sei();  // Globale Interrupts aktivieren

    brightness_stage = 2; // Starthelligkeit mittel
    OCR1A = brightness_levels[brightness_stage]; // Set initial brightness level
    OCR1B = brightness_levels[brightness_stage]; // Set initial brightness level
    update_display(); // Erste Anzeige updaten

    while (1) {
        asm("nop");
	    enter_sleep(); // Sleep Modus aktivieren und auf Interrupt warten
        // CPU wacht hier nach Interrupt auf
        _delay_ms(5); // Optional: Debounce oder Wake-up Handling
    }

    return 0;
}
