// --- Timer2 Overflow Interrupt Service Routine ---
ISR(TIMER2_OVF_vect) {
    static uint16_t timer2_tick_counter = 0; // Zähler für langsame Zeitmessung (Sekunden)

    // --- Nicht-blockierender Debounce für Buttons ---
    uint8_t current_button_state[3];
    current_button_state[0] = (PIND & (1 << MIN_BUTTON)) ? 1 : 0;
    current_button_state[1] = (PIND & (1 << HOUR_BUTTON)) ? 1 : 0;
    current_button_state[2] = (PIND & (1 << BRIGHTNESS_BUTTON)) ? 1 : 0;

    for (uint8_t i = 0; i < 3; i++) {
        if (current_button_state[i] != last_button_state[i]) {
            debounce_timer[i] = 0; // Reset Timer bei Zustandsänderung
        } else if (debounce_timer[i] < debounce_delay) {
            debounce_timer[i]++;
        }

        if (debounce_timer[i] >= debounce_delay) {
            if (current_button_state[i] == 0 && last_button_state[i] == 1) { // Fallende Flanke (Button gedrückt)
                // --- Führe Button-Aktion *direkt hier* aus, basierend auf dem Button-Index 'i' ---
                if (i == 0) { // MIN_BUTTON
                    if (!min_button_held) { // Nur kurzer Tastendruck, wenn nicht gehalten
                        minutes++;
                        seconds = 0;
                        if (minutes >= 60) {
                            minutes = 0;
                            hours++;
                            if (hours >= 24) {
                                hours = 0;
                            }
                        }
                        update_display(); // Display aktualisieren
                    }
                } else if (i == 1) { // HOUR_BUTTON
                    hours++;
                    if (hours >= 24) {
                        hours = 0;
                    }
                    update_display(); // Display aktualisieren
                } else if (i == 2) { // BRIGHTNESS_BUTTON
                    brightness_stage = (brightness_stage + 1) % 4;
                    OCR1A = brightness_levels[brightness_stage];
                    OCR1B = brightness_levels[brightness_stage];
                }
            }
        }
        last_button_state[i] = current_button_state[i]; // Aktualisiere letzten Zustand
    }

    // --- Zeitmessung (Sekunden, Minuten, Stunden) ---
    timer2_tick_counter++;
    if (timer2_tick_counter >= 16) { //  Anpassung des Zählers für ~1 Sekunde (ungefähr, muss feinjustiert werden)
        timer2_tick_counter = 0;

        // Increment seconds
        seconds++;

        // Handle minute button hold timing (bleibt wie gehabt)
        if (min_button_held) {
            min_hold_timer++;
            if (min_hold_timer >= 1) { // ~1 second hold
                minutes--;
                seconds = 0;
                min_button_held = 0; // Stop further timing
            }
        }

        // Increment minutes and hours as needed (bleibt wie gehabt)
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

        // Update the LED display (bleibt wie gehabt)
        update_display();
    }
}





// --- External Interrupt 0 Service Routine (HOUR_BUTTON) ---
ISR(INT0_vect) {
    // Interrupt einfach quittieren, Debounce und Aktion in Timer2 ISR
}

// --- External Interrupt 1 Service Routine (BRIGHTNESS_BUTTON) ---
ISR(INT1_vect) {
    // Interrupt einfach quittieren, Debounce und Aktion in Timer2 ISR
}

// --- Pin Change Interrupt 2 Service Routine (MIN_BUTTON) ---
ISR(PCINT2_vect) {
    // Hold-Timer-Logik für MIN_BUTTON bleibt hier
    if (!(PIND & (1 << MIN_BUTTON))) { // Button pressed
        min_button_held = 1; // Start hold timing
        min_hold_timer = 0;  // Reset timer
    } else { // Button released
        if ((PIND & (1 << MIN_BUTTON))) { // Confirm button is released
            if (min_button_held && min_hold_timer < 1) { // Short press
                // Kurzer Tastendruck wird nun in Timer2 ISR behandelt
            }
            min_button_held = 0; // Reset hold flag
            // Display Update wird nun in Timer2 ISR gemacht
        }
    }
}
