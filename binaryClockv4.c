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
#define HOUR_BUTTON     PD2
#define BRIGHTNESS_BUTTON PD3

// --- Debouncing Konstanten und Variablen ---
#define NUM_BUTTONS 3
#define DEBOUNCE_TICKS 50  // Anzahl Timer-Ticks für Debounce (z.B. 50ms bei 1ms Timer-Interrupt)
#define HOLD_TICKS_SHORT 100 // Anzahl Timer-Ticks für kurzen Hold (z.B. 100ms = 0.1s)
#define HOLD_TICKS_LONG  2000 // Anzahl Timer-Ticks für langen Hold (z.B. 2000ms = 2s)

volatile uint8_t button_state[NUM_BUTTONS];         // Aktueller entprellter Zustand der Buttons (0=losgelassen, 1=gedrückt)
volatile uint8_t button_raw_state[NUM_BUTTONS];     // Rohzustand der Button Pins
volatile uint16_t button_debounce_counter[NUM_BUTTONS]; // Debounce Zähler für jeden Button
volatile uint16_t button_hold_timer[NUM_BUTTONS];     // Hold Timer für jeden Button
volatile uint8_t button_held_short[NUM_BUTTONS];      // Flag für kurzen Tastendruck gehalten
volatile uint8_t button_held_long[NUM_BUTTONS];       // Flag für langen Tastendruck gehalten

// --- Zeit Variablen ---
volatile uint8_t seconds = 0;
volatile uint8_t minutes = 0;
volatile uint8_t hours = 0;

// --- Helligkeit Variablen ---
volatile uint8_t brightness_stage = 0;
const uint8_t brightness_levels[] = {254, 248, 156, 0}; // Define brightness levels (in reversed order)


void setup_timer2() {
    // Timer2 für Zeitmessung (interner Takt)
    TCCR2A = 0;
    TCCR2B = (1 << CS22) | (1 << CS20); // Prescaler 128
    TIMSK2 = (1 << TOIE2);           // Overflow Interrupt aktivieren
    while (ASSR & ((1 << TCR2AUB) | (1 << TCR2BUB))); // Warten auf Stabilisierung
}


void setup_timer1() {
    // Timer1 für PWM
    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1); // Fast PWM, Clear OC1A/B on Compare Match
    TCCR1B = (1 << WGM12) | (1 << CS11);                   // Prescaler 8
    OCR1A = brightness_levels[brightness_stage]; // Initiale Helligkeit Stunden LEDs
    OCR1B = brightness_levels[brightness_stage]; // Initiale Helligkeit Minuten LEDs
}


void setup_timer0_debounce() {
    // Timer0 für Debouncing (CTC Modus)
    TCCR0A = (1 << WGM01);        // CTC Modus
    TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64 (ca. 1ms Interrupt bei 8MHz)
    OCR0A = 124;                  // Compare Value für ca. 1ms Interrupt (8MHz / 64 / 1000 = 125 -1)
    TIMSK0 |= (1 << OCIE0A);       // Compare Match A Interrupt aktivieren
}


void setup_ports() {
    // Stunden LEDs als Ausgänge
    DDRD |= (1 << HOUR_LED_BIT0) | (1 << HOUR_LED_BIT1) | (1 << HOUR_LED_BIT2) | (1 << HOUR_LED_BIT3) | (1 << HOUR_LED_BIT4);
    // Minuten LEDs als Ausgänge
    DDRC |= 0x3F; // PC0-PC5
    // PWM Kathodensteuerung als Ausgänge
    DDRB |= (1 << PB1) | (1 << PB2);

    // Buttons als Eingänge mit Pull-Up
    DDRD &= ~((1 << MIN_BUTTON) | (1 << HOUR_BUTTON) | (1 << BRIGHTNESS_BUTTON));
    PORTD |= (1 << MIN_BUTTON) | (1 << HOUR_BUTTON) | (1 << BRIGHTNESS_BUTTON);

    // Alle anderen PORTB Pins als Eingang mit Pull-Up (optional, falls benötigt)
    DDRB &= ~((1 << PB0) | (1 << PB3) | (1 << PB4) | (1 << PB5) | (1 << PB6) | (1 << PB7));
    PORTB |=  ((1 << PB0) | (1 << PB3) | (1 << PB4) | (1 << PB5) | (1 << PB6) | (1 << PB7));


    // LEDs initial aus
    HOUR_LED_PORT &= ~((1 << HOUR_LED_BIT0) | (1 << HOUR_LED_BIT1) | (1 << HOUR_LED_BIT2) | (1 << HOUR_LED_BIT3) | (1 << HOUR_LED_BIT4));
    MINUTE_LED_PORT &= ~0x3F;
    OCR1A = 0; // PWM aus
    OCR1B = 0; // PWM aus
}


void setup_interrupts() {
    // Pin Change Interrupt für Buttons auf PORTD aktivieren
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT17) | (1 << PCINT18) | (1 << PCINT19); // PD1, PD2, PD3 (MIN_BUTTON, HOUR_BUTTON, BRIGHTNESS_BUTTON)
}


void update_display() {
    // Stunden LEDs setzen
    HOUR_LED_PORT &= ~((1 << HOUR_LED_BIT0) | (1 << HOUR_LED_BIT1) | (1 << HOUR_LED_BIT2) | (1 << HOUR_LED_BIT3) | (1 << HOUR_LED_BIT4));
    if (hours & (1 << 0)) HOUR_LED_PORT |= (1 << HOUR_LED_BIT0); // Stunde 1 (PD0)
    if (hours & (1 << 1)) HOUR_LED_PORT |= (1 << HOUR_LED_BIT1); // Stunde 2 (PD7)
    if (hours & (1 << 2)) HOUR_LED_PORT |= (1 << HOUR_LED_BIT2); // Stunde 4 (PD4)
    if (hours & (1 << 3)) HOUR_LED_PORT |= (1 << HOUR_LED_BIT3); // Stunde 8 (PD5)
    if (hours & (1 << 4)) HOUR_LED_PORT |= (1 << HOUR_LED_BIT4); // Stunde 16 (PD6)

    // Minuten LEDs setzen
    MINUTE_LED_PORT &= ~0x3F;
    MINUTE_LED_PORT |= (minutes & 0x3F);
}


ISR(TIMER2_OVF_vect) {
    // Sekunden inkrementieren (Zeitbasis)
    seconds++;

    if (seconds >= 60) {
        seconds = 0;
        minutes++;
        if (minutes >= 60) {
            minutes = 0;
            hours++;
            if (hours >= 24) {
                hours = 0;
            }
        }
    }
    update_display(); // Anzeige aktualisieren
}


ISR(TIMER0_COMPA_vect) {
    // --- Timer0 Compare Match ISR (für Debouncing und Button-Aktionen) ---
    uint8_t current_button_raw_state[NUM_BUTTONS];

    // Rohzustand der Buttons lesen
    current_button_raw_state[0] = !(PIND & (1 << MIN_BUTTON));       // Button 1 (MIN_BUTTON)
    current_button_raw_state[1] = !(PIND & (1 << HOUR_BUTTON));      // Button 2 (HOUR_BUTTON)
    current_button_raw_state[2] = !(PIND & (1 << BRIGHTNESS_BUTTON)); // Button 3 (BRIGHTNESS_BUTTON)


    for (int i = 0; i < NUM_BUTTONS; i++) {
        if (current_button_raw_state[i]) { // Button Pin ist LOW (gedrückt)
            if (button_debounce_counter[i] < DEBOUNCE_TICKS) {
                button_debounce_counter[i]++;
            }
            if (button_debounce_counter[i] >= DEBOUNCE_TICKS) {
                if (!button_state[i]) { // Zustand hat sich von losgelassen zu gedrückt geändert
                    button_state[i] = 1;
                    button_hold_timer[i] = 0; // Hold Timer zurücksetzen bei Tastendruck
                    button_held_short[i] = 0;  // Reset short hold flag
                    button_held_long[i] = 0;   // Reset long hold flag
                } else { // Button ist weiterhin gedrückt
                    button_hold_timer[i]++;
                    if (button_hold_timer[i] == HOLD_TICKS_SHORT) {
                        button_held_short[i] = 1; // Short Hold Flag setzen
                    }
                    if (button_hold_timer[i] == HOLD_TICKS_LONG) {
                        button_held_long[i] = 1;  // Long Hold Flag setzen
                    }
                }
            }
        } else { // Button Pin ist HIGH (losgelassen)
            if (button_debounce_counter[i] > 0) {
                button_debounce_counter[i]--;
            }
            if (button_debounce_counter[i] == 0) {
                if (button_state[i]) { // Zustand hat sich von gedrückt zu losgelassen geändert
                    button_state[i] = 0;
                    if (!button_held_short[i] && !button_held_long[i]) {
                        // --- Kurzer Tastendruck Aktion ---
                        if (i == 0) { // MIN_BUTTON (Button 1)
                            minutes++;
                            seconds = 0;
                            if (minutes >= 60) {
                                minutes = 0;
                                hours++;
                                if (hours >= 24) hours = 0;
                            }
                            update_display();
                        } else if (i == 1) { // HOUR_BUTTON (Button 2)
                            hours++;
                            if (hours >= 24) hours = 0;
                            update_display();
                        } else if (i == 2) { // BRIGHTNESS_BUTTON (Button 3)
                            brightness_stage = (brightness_stage + 1) % 4;
                            OCR1A = brightness_levels[brightness_stage];
                            OCR1B = brightness_levels[brightness_stage];
                        }
                    }
                    button_held_short[i] = 0; // Zurücksetzen beim Loslassen
                    button_held_long[i] = 0;  // Zurücksetzen beim Loslassen
                }
            }
        }
    }

    // --- Hold Aktionen (werden kontinuierlich ausgeführt, solange Button gehalten) ---
    if (button_state[0] && button_held_short[0]) { // MIN_BUTTON gehalten (kurz)
        // Schnellere Minuten Inkrementierung (Beispiel)
        if ((button_hold_timer[0] % (HOLD_TICKS_SHORT / 2)) == 0) { // Alle 0.5 Sekunden (Beispiel)
            minutes++;
            seconds = 0;
            if (minutes >= 60) {
                minutes = 0;
                hours++;
                if (hours >= 24) hours = 0;
            }
            update_display();
        }
    }
    if (button_state[1] && button_held_short[1]) { // HOUR_BUTTON gehalten (kurz)
        // Schnellere Stunden Inkrementierung (Beispiel)
        if ((button_hold_timer[1] % (HOLD_TICKS_SHORT / 2)) == 0) { // Alle 0.5 Sekunden (Beispiel)
            hours++;
            if (hours >= 24) hours = 0;
            update_display();
        }
    }
    if (button_state[2] && button_held_long[2]) { // BRIGHTNESS_BUTTON lange gehalten
        // Beispiel: Helligkeit auf Maximum setzen bei langem Hold
        OCR1A = brightness_levels[0]; // Maximale Helligkeit
        OCR1B = brightness_levels[0];
        brightness_stage = 0; // Brightness Stage anpassen
    }
}


ISR(PCINT2_vect) {
    // Pin Change Interrupt für PORTD (Buttons)
    // Hier passiert fast nichts mehr, da Debouncing und Button-Aktionen im Timer0 ISR stattfinden
    // Wir könnten hier höchstens die rohen Button-Zustände schneller erfassen, falls nötig,
    // aber für die hier verwendete Debouncing-Methode ist das nicht unbedingt notwendig.
    // Die rohen Zustände werden bereits im Timer0 ISR regelmäßig gelesen.
}


void enter_sleep() {
    if(OCR1A == 0 && OCR1B == 0) {
        set_sleep_mode(SLEEP_MODE_PWR_SAVE);
        sleep_enable();
        sleep_cpu();
    } else {
	    set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_enable();
        sleep_cpu();
    }
    sleep_disable();
}


int main() {
    setup_ports();
    setup_timer2();
    setup_timer1();
    setup_timer0_debounce(); // Timer0 für Debouncing initialisieren
    setup_interrupts();

    sei();

    // Initialisiere Button-Zustände und Debounce-Zähler
    for (int i = 0; i < NUM_BUTTONS; i++) {
        button_state[i] = 0;
        button_debounce_counter[i] = 0;
        button_hold_timer[i] = 0;
        button_held_short[i] = 0;
        button_held_long[i] = 0;
    }

    brightness_stage = 2; // Starthelligkeit mittel
    OCR1A = brightness_levels[brightness_stage];
    OCR1B = brightness_levels[brightness_stage];
    update_display();


    while (1) {
        asm("nop");
	    enter_sleep();
    }

    return 0;
}