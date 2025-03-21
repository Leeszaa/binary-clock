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


volatile uint8_t seconds = 0;
volatile uint8_t minutes = 20;
volatile uint8_t hours = 5;
volatile uint8_t brightness_stage = 0; 	   // Tracks the current brightness stage (0-3)
volatile uint8_t pwm_hold_timer = 0;       // Tracks how long the brightness button is held
volatile uint8_t pwm_button_held = 0;      // Flag indicating if the button is currently held
volatile uint8_t min_hold_timer = 0;	   // Times how long the min button is hold
volatile uint8_t min_button_held = 0;	   // Flags min button as held
volatile uint8_t hour_hold_timer = 0;	   // Times how long the hour button is hold
volatile uint8_t hour_button_held = 0;	   // Flags hour button as held

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

    // --- Pin Change Interrupt für MIN_BUTTON (PD1) beibehalten ---
    PCICR |= (1 << PCIE2); // Pin-Change Interrupt für PORTD aktivieren
    PCMSK2 |= (1 << PCINT17); // PD1 (MIN_BUTTON) als Interrupt Pin (PCINT17)
    // PCMSK2 &= ~((1 << PCINT18) | (1 << PCINT19)); // PD2 und PD3 NICHT mehr als PCINT, da INT0/INT1 verwendet werden
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
    PORTC |= (minutes & 0x3F); // Minuten direkt auf PC0-PC5 schreiben, da sie sequenziell sind.
}


ISR(TIMER2_OVF_vect) {
    // Increment seconds
    seconds++;

    // Handle button hold timing for brightness button (PD3) - ISR(INT1_vect) handles brightness button directly now, this part is not used anymore for direct button press detection but hold functionality could be kept if needed for brightness
    /*if (pwm_button_held) {
        pwm_hold_timer++;
        if (pwm_hold_timer >= 2) { // ~2 Sekunden halten
            OCR1A = 255;
            OCR1B = 255;
            pwm_button_held = 0; // Stop further timing
	        brightness_stage = (brightness_stage - 1) % 4;
	        OCR1A = brightness_levels[brightness_stage];
            OCR1B = brightness_levels[brightness_stage];
        }
    }*/
    if (min_button_held) {
        min_hold_timer++;
        if (min_hold_timer >= 1) { // ~1 Sekunden halten
	        minutes--;
	        seconds = 0;
            min_button_held = 0; // Stop further timing
        }
    }
    /*if (hour_button_held) {
        hour_hold_timer++;
        if (hour_hold_timer >= 1) { // ~1 Sekunden halten
	        hours--;
            hour_button_held = 0; // Stop further timing
        }
    }*/


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

ISR(INT0_vect) {
    // ISR für INT0 (HOUR_BUTTON - PD2)
    // Direct handling of HOUR_BUTTON press
    _delay_ms(5); // Debounce
    if (!(PIND & (1 << HOUR_BUTTON))) { // Bestätige Button ist immer noch gedrückt
        hours++;
        if (hours >= 24) {
            hours = 0;
        }
        update_display();
    }
}

ISR(INT1_vect) {
    // ISR für INT1 (BRIGHTNESS_BUTTON - PD3)
    // Direct handling of BRIGHTNESS_BUTTON press
    _delay_ms(5); // Debounce
    if (!(PIND & (1 << BRIGHTNESS_BUTTON))) { // Bestätige Button ist immer noch gedrückt
        brightness_stage = (brightness_stage + 1) % 4;
        OCR1A = brightness_levels[brightness_stage];
        OCR1B = brightness_levels[brightness_stage];
    }
}


ISR(PCINT2_vect) {
    // ISR für PCINT2 (wird jetzt nur noch für MIN_BUTTON - PD1 verwendet)
    // Handle PD1 button (Increment minutes) - SW1
    if ( !(PIND & (1 << PD1))) { // Button gedrückt
        _delay_ms(5); // Debounce
        if (!(PIND & (1 << PD1))) { // Bestätige Button ist immer noch gedrückt
            min_button_held = 1; // Hold Timing starten
            min_hold_timer = 0;  // Timer zurücksetzen
        }
    } else { // Button losgelassen
        _delay_ms(5); // Debounce
        if ((PIND & (1 << PD1))) { // Bestätige Button ist losgelassen
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
	        min_button_held = 0; // Hold Flag zurücksetzen
            update_display(); // Sofort updaten
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
