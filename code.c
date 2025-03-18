/*
binary_clock_dcf77 by Ben Kaden (erweitert - KOMPLETT)
Binary Clock mit DCF77 Integration - Vollständige Implementierung
*/

// !! Konfiguration DCF77 !!
#define DCF77_PIN PD2
#define DCF77_PIN_BIT (1 << DCF77_PIN)
#define DCF77_INTERRUPT PCINT2_vect // Interrupt für PORTD (PCINT2)
#define DCF77_PCIE PCIE2          // PCIE2 für PORTD Pin Change Interrupt Enable
#define DCF77_PCMSK PCMSK2        // PCMSK2 für PORTD Pin Change Mask Register
#define DCF77_PCINT_BIT PCINT18     // PCINT18 für PD2 (PCINT-Nummer für PD2)

#define DCF77_TIMEOUT_MS 8000       // Timeout für DCF77-Signalempfang (8 Sekunden - etwas länger für vollständigen Frame)
#define DCF77_SYNC_INTERVAL_MS (60 * 60 * 1000) // Synchronisiere jede Stunde (3600000ms)

// DCF77 Pulsdauern (ungefähre Werte in Millisekunden)
#define DCF77_PULSE_0_MIN 70
#define DCF77_PULSE_0_MAX 150
#define DCF77_PULSE_1_MIN 170
#define DCF77_PULSE_1_MAX 280
#define DCF77_PULSE_MINUTE_MARKER_MIN 450
#define DCF77_PULSE_MINUTE_MARKER_MAX 700

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <stdbool.h> // Für boolesche Datentypen

// Zeitvariablen (volatile für ISR-Zugriff)
volatile uint8_t currentSeconds = 0;
volatile uint8_t currentMinutes = 0;
volatile uint8_t currentHours = 12;
volatile uint8_t currentDay = 1;     // Tag des Monats
volatile uint8_t currentMonth = 1;   // Monat (1-12)
volatile uint8_t currentYear = 24;   // Jahr (letzte zwei Ziffern, z.B. 2024)
volatile uint8_t currentWeekday = 1; // Wochentag (1=Montag, 7=Sonntag)

// ... (Helligkeitsvariablen und Button-Variablen bleiben gleich) ...
volatile uint8_t currentBrightnessLevelIndex = 0;
volatile uint8_t brightnessButtonHoldTimer = 0;
volatile bool isBrightnessButtonHeld = false;
volatile uint8_t minutesButtonHoldTimer = 0;
volatile bool isMinutesButtonHeld = false;
volatile uint8_t hoursButtonHoldTimer = 0;
volatile bool isHoursButtonHeld = false;

const uint8_t brightnessLevels[] = {254, 248, 156, 0};

// DCF77-bezogene Variablen
volatile bool dcf77SignalReceived = false;
volatile uint8_t dcf77Data[60]; // Puffer für DCF77 Daten (60 Bits pro Minute)
volatile uint8_t dcf77BitIndex = 0;
volatile uint16_t dcf77TimeoutTimer = 0;
volatile bool dcf77Receiving = false;
volatile bool dcf77SyncSuccessful = false;
volatile uint32_t lastDcf77SyncTime = 0; // Zeit des letzten erfolgreichen DCF77-Syncs
volatile uint32_t dcf77PulseStartTime = 0; // Startzeit des aktuellen Pulses (für Pulsmessung)

// Funktion Prototypen
void initializeTimekeepingTimer(void);
void initializePWM(void);
void initializeIO(void);
void initializeInterrupts(void);
void updateBinaryDisplay(void);
void enterSleepMode(void);

// DCF77 Funktionen Prototypen
void initializeDcf77(void);
bool decodeDcf77Signal(void);
bool parseDcf77Data(void); // Funktion zum Parsen der empfangenen Daten
bool validateDcf77Data(void); // Funktion zur Validierung der DCF77 Daten (Parität)
void synchronizeTimeFromDcf77(uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t day, uint8_t month, uint8_t year, uint8_t weekday);
uint32_t millis(void);
void initializeMillisTimer(void);

// Initialize Timer2 for timekeeping (using asynchronous mode for 32.768 kHz crystal if available)
void initializeTimekeepingTimer(void) {
    // Activate asynchronous mode for Timer2 using external clock source (if available)
    ASSR |= (1 << AS2);

    // Set Timer2 to Normal mode (simplest counting mode)
    TCCR2A = 0;
    TCCR2B = (1 << CS22) | (1 << CS20); // Prescaler = 128 for 1 second increments with 32.768 kHz crystal (adjust if using internal oscillator)

    // Enable Timer2 Overflow interrupt to trigger time updates
    TIMSK2 = (1 << TOIE2);

    // Wait for asynchronous clock to stabilize after configuration
    while (ASSR & ((1 << TCR2AUB) | (1 << TCR2BUB))) {
        // Wait until Timer2 registers are updated asynchronously
    }
}

// Initialize Timer1 for PWM control of LEDs for brightness adjustment
void initializePWM(void) {
    // Configure Timer1 for Fast PWM mode, 8-bit resolution, non-inverting PWM on OC1A and OC1B
    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1); // Mode 5 (Fast PWM 8-bit), Clear OC1A/OC1B on Compare Match, set at BOTTOM
    TCCR1B = (1 << WGM12) | (1 << CS11);                  // Mode 5 (Fast PWM 8-bit), Prescaler = 8 for reasonable PWM frequency

    // Set initial PWM duty cycle based on the initial brightness level
    OCR1A = brightnessLevels[currentBrightnessLevelIndex];
    OCR1B = brightnessLevels[currentBrightnessLevelIndex];
}

// Initialize Input/Output ports: LED outputs, button inputs, and peripheral power reduction
void initializeIO(void) {
    // Configure PORTC (PC0-PC4) and PORTD (PD0-PD5) as outputs for hours and minutes LEDs respectively
    DDRC |= 0x1F; // PC0 to PC4 as outputs (5 bits for hours up to 23 - binary representation)
    DDRD |= 0x3F; // PD0 to PD5 as outputs (6 bits for minutes up to 59 - binary representation)

    // Disable unused peripherals to save power (ADC, USART, SPI, Timer0, TWI)
    PRR |= (1 << PRADC) | (1 << PRUSART0) | (1 << PRSPI) | (1 << PRTIM0) | (1 << PRTWI);

    // Configure PORTB pins as inputs with pull-up resistors enabled (for buttons)
    DDRB = 0x00;  // All PORTB pins as inputs
    PORTB = 0xFF; // Enable pull-up resistors on all PORTB pins

    // Turn off LEDs initially by setting output ports low
    PORTC &= ~0x1F;
    PORTD &= ~0x3F;

    // Configure PB1 and PB2 as outputs for PWM signals to control LED brightness
    DDRB |= (1 << PB1) | (1 << PB2);

    // Configure button pins (PB0, PD6, PD7) as inputs with pull-up resistors (already enabled above for PORTB/D)
    DDRB &= ~(1 << PB0);          // PB0 (Minutes button) as input
    DDRD &= ~((1 << PD6) | (1 << PD7) | (1 << DCF77_PIN)); // PD6 (Hours button), PD7 (Brightness/Sleep button), DCF77_PIN as inputs - DCF77 Pin hinzugefügt
    // Pull-up resistors are already enabled for PORTB and PORTD
    PORTD |= (1 << DCF77_PIN); // Pull-up für DCF77 Pin aktivieren (falls benötigt, abhängig vom Modul) - Hinzugefügt
}

// Initialize external interrupts for button presses (using Pin Change Interrupts)
void initializeInterrupts(void) {
    // ... (vorherige Interrupt-Initialisierung für Buttons) ...
    PCICR |= (1 << PCIE0) | (1 << PCIE2);
    PCMSK0 |= (1 << PCINT0);
    PCMSK2 |= (1 << PCINT22) | (1 << PCINT23) | (1 << DCF77_PCINT_BIT); // DCF77 Pin zu PCMSK2 hinzufügen
    // DCF77 Pin als Eingang konfigurieren (sollte bereits in initializeIO passieren, aber sicherstellen) - Sichergestellt in initializeIO
}

// Update the binary LED display to show current hours and minutes
void updateBinaryDisplay(void) {
    // Display hours on PORTC (PC0-PC4) -  using bitwise AND to clear previous hour value and then ORing with new hour value
    PORTC = (PORTC & ~0x1F) | (currentHours & 0x1F); // Keep other bits of PORTC unchanged

    // Display minutes on PORTD (PD0-PD5) - using bitwise AND to clear previous minute value and then ORing with new minute value
    PORTD = (PORTD & ~0x3F) | (currentMinutes & 0x3F); // Keep other bits of PORTD unchanged
}

// Function to enter sleep mode (triggered by long press of brightness button or low brightness level)
void enterSleepMode(void) {
    if (currentBrightnessLevelIndex == 3) { // If brightness is set to off (level 3)
        set_sleep_mode(SLEEP_MODE_PWR_SAVE); // Deeper sleep mode for maximum power saving (disables most peripherals except asynchronous timer)
        sleep_enable();                      // Enable sleep mode
        sleep_cpu();                         // Enter sleep - program execution pauses here until wake-up interrupt
    } else {
        set_sleep_mode(SLEEP_MODE_IDLE);   // Idle sleep mode - CPU clock is stopped, but peripherals are still running (less power saving, faster wake-up)
        sleep_enable();                      // Enable sleep mode
        sleep_cpu();                         // Enter sleep - program execution pauses here until wake-up interrupt
    }
    // Program execution resumes here after wake-up from sleep (due to timer overflow or button press interrupt)
    sleep_disable(); // Disable sleep mode immediately after wake-up
}


// Timer2 Overflow ISR (Zeitaktualisierung und DCF77 Timeout)
ISR(TIMER2_OVF_vect) {
    // ... (vorherige Zeitaktualisierung und Button-Hold-Timer Logik) ...
    // Button hold timer logic - increment timers if buttons are held
    if (isBrightnessButtonHeld) {
        brightnessButtonHoldTimer++;
        if (brightnessButtonHoldTimer >= 2) { // Approximately 2 seconds hold for sleep mode activation
            OCR1A = 255; // Set PWM to 255 (effectively turn off PWM for some LED configurations, or set to very low brightness depending on wiring)
            OCR1B = 255;
            isBrightnessButtonHeld = false; // Stop timer and flag after long press action is triggered
            currentBrightnessLevelIndex = 3; // Force brightness level index to off
            //enterSleepMode(); // Consider moving sleep entry to main loop based on brightness level
        }
    }
    if (isMinutesButtonHeld) {
        minutesButtonHoldTimer++;
        if (minutesButtonHoldTimer >= 1) { // Approximately 1 second hold for continuous minute adjustment
            currentMinutes--;
            currentSeconds = 0; // Reset seconds when minutes are adjusted to maintain accuracy
            isMinutesButtonHeld = false; // Stop timer and flag after long press action is triggered
        }
    }
    if (isHoursButtonHeld) {
        hoursButtonHoldTimer++;
        if (hoursButtonHoldTimer >= 1) { // Approximately 1 second hold for continuous hour adjustment
            currentHours--;
            isHoursButtonHeld = false;   // Stop timer and flag after long press action is triggered
        }
    }


    // DCF77 Timeout-Zähler inkrementieren, falls Empfang aktiv ist
    if (dcf77Receiving) {
        dcf77TimeoutTimer++;
        if (dcf77TimeoutTimer >= (DCF77_TIMEOUT_MS / 10)) { // Timeout erreicht (Timer2 ISR läuft ca. alle 10ms)
            dcf77Receiving = false; // DCF77 Empfang abbrechen
            dcf77SignalReceived = false; // Signal als nicht empfangen markieren
            dcf77SyncSuccessful = false; // Sync als fehlgeschlagen markieren
            // Optional: Fehler-LED anzeigen
        }
    }

    // Zeit aktualisieren (wie zuvor) - läuft auch ohne DCF77 weiter
    currentSeconds++;
    if (currentSeconds >= 60) {
        currentSeconds = 0;
        currentMinutes++;
        if (currentMinutes >= 60) {
            currentMinutes = 0;
            currentHours++;
            if (currentHours >= 24) {
                currentHours = 0;
            }
        }
    }
    updateBinaryDisplay();
}

// Pin Change Interrupt ISR für PORTB (handles minutes button - PB0)
ISR(PCINT0_vect) {
    // Handle Minutes Button (PB0) press and release
    if (!(PINB & (1 << PB0))) { // Button is pressed (logic low due to pull-up)
        _delay_ms(5); // Debounce delay to avoid spurious readings
        if (!(PINB & (1 << PB0))) { // Confirm button is still pressed after debounce
            isMinutesButtonHeld = true; // Set flag to indicate button is being held for long press detection
            minutesButtonHoldTimer = 0;  // Reset hold timer when button is initially pressed
        }
    } else { // Button is released (logic high due to pull-up)
        _delay_ms(5); // Debounce delay for release
        if ((PINB & (1 << PB0))) { // Confirm button is still released after debounce
            if (isMinutesButtonHeld && minutesButtonHoldTimer < 1) { // Short press detected (held flag set but timer did not reach long press threshold)
                currentMinutes++; // Increment minutes on short press
                currentSeconds = 0; // Reset seconds to 0 when minutes are adjusted
                if (currentMinutes >= 60) {
                    currentMinutes = 0;
                    currentHours++;
                    if (currentHours >= 24) {
                        currentHours = 0;
                    }
                }
            }
            isMinutesButtonHeld = false; // Reset hold flag on button release
            updateBinaryDisplay();       // Update display immediately after button action
        }
    }
}

// Pin Change Interrupt ISR für PORTD (Buttons und DCF77)
ISR(PCINT2_vect) {
    static uint32_t lastPulseEndTime = 0; // Statische Variable, um das Ende des vorherigen Pulses zu speichern

    // Handle Hours Button (PD6) press and release
    if (!(PIND & (1 << PD6))) { // Hours button pressed
        _delay_ms(5); // Debounce
        if (!(PIND & (1 << PD6))) { // Confirm press
            isHoursButtonHeld = true;  // Set hold flag for hours button
            hoursButtonHoldTimer = 0;   // Reset hold timer for hours button
        }
    } else { // Hours button released
        _delay_ms(5); // Debounce
        if ((PIND & (1 << PD6))) { // Confirm release
            if (isHoursButtonHeld && hoursButtonHoldTimer < 1) { // Short press on hours button
                currentHours++; // Increment hours
                if (currentHours >= 24) {
                    currentHours = 0;
                }
            }
            isHoursButtonHeld = false; // Reset hours button hold flag
            updateBinaryDisplay();      // Update display
        }
    }

    // Handle Brightness/Sleep Button (PD7) press and release
    if (!(PIND & (1 << PD7))) { // Brightness/Sleep button pressed
        _delay_ms(5); // Debounce
        if (!(PIND & (1 << PD7))) { // Confirm press
            isBrightnessButtonHeld = true; // Set hold flag for brightness button
            brightnessButtonHoldTimer = 0;  // Reset hold timer for brightness button
        }
    } else { // Brightness/Sleep button released
        _delay_ms(5); // Debounce
        if ((PIND & (1 << PD7))) { // Confirm release
            if (isBrightnessButtonHeld && brightnessButtonHoldTimer < 2) { // Short press on brightness button
                currentBrightnessLevelIndex = (currentBrightnessLevelIndex + 1) % 4; // Cycle through brightness levels
                OCR1A = brightnessLevels[currentBrightnessLevelIndex]; // Update PWM duty cycle for both channels
                OCR1B = brightnessLevels[currentBrightnessLevelIndex];
            }
            isBrightnessButtonHeld = false; // Reset brightness button hold flag
        }
    }

    // DCF77 Signalverarbeitung (PD2)
    if (PIND & DCF77_PIN_BIT) { // Pegelwechsel an DCF77_PIN (PD2) detektiert (FALLENDE Flanke - Signal ist invertiert)
        uint32_t currentTime = millis();
        uint32_t pulseDuration = currentTime - lastPulseEndTime; // Pulsdauer seit letzter Flanke messen

        if (!dcf77Receiving) {
            dcf77Receiving = true; // Empfang starten beim ersten Puls
            dcf77TimeoutTimer = 0; // Timeout-Timer zurücksetzen
            dcf77BitIndex = 0;     // Bit-Index zurücksetzen
            dcf77SignalReceived = false; // Flag zurücksetzen
            dcf77SyncSuccessful = false; // Sync-Flag zurücksetzen
            // Optional: Empfangs-LED einschalten
        } else {
            decodeDcf77Signal(pulseDuration); // DCF77 Signal dekodieren und Pulsdauer übergeben
        }
        lastPulseEndTime = currentTime; // Aktuelle Zeit als Ende des Pulses speichern für nächste Messung
    }
}


// DCF77 Signal Dekodierungsfunktion
bool decodeDcf77Signal(uint32_t pulseDuration) {
    if (!dcf77Receiving) return false; // Nicht dekodieren, wenn Empfang nicht aktiv

    if (pulseDuration > DCF77_PULSE_MINUTE_MARKER_MIN && pulseDuration < DCF77_PULSE_MINUTE_MARKER_MAX) {
        // Minutenmarker erkannt (Start einer neuen Minute)
        dcf77BitIndex = 0; // Bit-Index zurücksetzen für neuen Datenframe
        dcf77Data[dcf77BitIndex++] = 2; // Minute Marker kennzeichnen (z.B. mit Wert 2)
    } else if (pulseDuration > DCF77_PULSE_1_MIN && pulseDuration < DCF77_PULSE_1_MAX) {
        // 1-Bit erkannt
        if (dcf77BitIndex < 60) dcf77Data[dcf77BitIndex++] = 1;
    } else if (pulseDuration > DCF77_PULSE_0_MIN && pulseDuration < DCF77_PULSE_0_MAX) {
        // 0-Bit erkannt
        if (dcf77BitIndex < 60) dcf77Data[dcf77BitIndex++] = 0;
    } else {
        // Ungültige Pulsdauer oder Rauschen - Empfang abbrechen
        dcf77Receiving = false;
        dcf77SignalReceived = false;
        dcf77SyncSuccessful = false;
        return false;
    }

    if (dcf77BitIndex >= 60) { // Vollständiger Datenframe empfangen (60 Bits + Minute Marker)
        dcf77Receiving = false;
        dcf77SignalReceived = true;
        if (parseDcf77Data()) { // Daten parsen und Zeit synchronisieren
            dcf77SyncSuccessful = true; // Sync erfolgreich
            lastDcf77SyncTime = millis(); // Zeit des letzten erfolgreichen Syncs merken
            // Optional: Erfolgs-LED anzeigen
            return true;
        } else {
            dcf77SyncSuccessful = false; // Sync fehlgeschlagen (Daten ungültig)
            // Optional: Fehler-LED anzeigen
            return false;
        }
    }
    return false; // Empfang läuft noch
}


// Funktion zum Parsen der DCF77 Daten und Extrahieren der Zeit
bool parseDcf77Data(void) {
    if (!dcf77SignalReceived) return false; // Keine Daten zum Parsen

    if (!validateDcf77Data()) { // Datenvalidierung fehlgeschlagen (Parität)
        return false;
    }

    // BCD-Dekodierung und Datenextraktion (DCF77 Datenformat beachten!)
    uint8_t minuteBCD = (dcf77Data[20] << 4) | (dcf77Data[19] << 3) | (dcf77Data[18] << 2) | (dcf77Data[17] << 1) | dcf77Data[16];
    uint8_t hourBCD   = (dcf77Data[26] << 4) | (dcf77Data[25] << 3) | (dcf77Data[24] << 2) | (dcf77Data[23] << 1) | dcf77Data[22];
    uint8_t dayBCD    = (dcf77Data[31] << 4) | (dcf77Data[30] << 3) | (dcf77Data[29] << 2) | (dcf77Data[28] << 1) | dcf77Data[27];
    uint8_t monthBCD  = (dcf77Data[35] << 4) | (dcf77Data[34] << 3) | (dcf77Data[33] << 2) | (dcf77Data[32] << 1);
    uint8_t yearBCD   = (dcf77Data[49] << 4) | (dcf77Data[48] << 3) | (dcf77Data[47] << 2) | (dcf77Data[46] << 1) | dcf77Data[45] ;
    uint8_t weekdayBCD = (dcf77Data[38] << 2) | (dcf77Data[37] << 1) | (dcf77Data[36];


    uint8_t minutes = (minuteBCD >> 4) * 10 + (minuteBCD & 0x0F);
    uint8_t hours   = (hourBCD >> 4) * 10 + (hourBCD & 0x0F);
    uint8_t day     = (dayBCD >> 4) * 10 + (dayBCD & 0x0F);
    uint8_t month   = (monthBCD >> 4) * 10 + (monthBCD & 0x0F); // Monat ist nur einstellig BCD
    uint8_t year    = (yearBCD >> 4) * 10 + (yearBCD & 0x0F);
    uint8_t weekday = (weekdayBCD >> 4) * 10 + (weekdayBCD & 0x0F);


    // Plausibilitätsprüfung (Bereichsprüfung) - wichtig!
    if (minutes > 59 || hours > 23 || day > 31 || month > 12 || weekday > 7 || weekday < 1) return false; // Ungültige Werte

    synchronizeTimeFromDcf77(hours, minutes, 0, day, month, year, weekday); // Sekunden auf 0 setzen beim Sync
    return true; // Erfolgreiches Parsen
}

// Funktion zur Validierung der DCF77 Daten (Paritätsprüfung)
bool validateDcf77Data(void) {
    if (!dcf77SignalReceived) return false;

    uint8_t parityMinute = 0;
    for (int i = 16; i <= 21; i++) parityMinute += dcf77Data[i];
    if ((parityMinute % 2) != dcf77Data[21]) return false; // Parität Minute falsch

    uint8_t parityHour = 0;
    for (int i = 22; i <= 27; i++) parityHour += dcf77Data[i];
    if ((parityHour % 2) != dcf77Data[28]) return false;   // Parität Stunde falsch

    uint8_t parityDate = 0;
    for (int i = 29; i <= 35; i++) parityDate += dcf77Data[i];
    for (int i = 42; i <= 49; i++) parityDate += dcf77Data[i]; // Year Parity auch hier prüfen (korrigiert)
    if ((parityDate % 2) != dcf77Data[50]) return false;   // Parität Datum/Jahr falsch


    // Keine Fehler gefunden, Daten sind valid
    return true;
}


// Funktion zum Synchronisieren der internen Uhr mit DCF77-Daten
void synchronizeTimeFromDcf77(uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t day, uint8_t month, uint8_t year, uint8_t weekday) {
    currentHours = hours;
    currentMinutes = minutes;
    currentSeconds = seconds;
    currentDay = day;
    currentMonth = month;
    currentYear = year;
    currentWeekday = weekday;
    // Optional: Datum, Wochentag etc. auch synchronisieren, falls in DCF77-Daten vorhanden und benötigt.
}


// millis() Funktion (genauer als zuvor)
static volatile uint32_t timer0_millis;
uint32_t millis() {
  return timer0_millis;
}

ISR(TIMER0_COMPA_vect) { // Timer0 Compare Match A Interrupt (besser für präzise Timings)
  timer0_millis++;
}

void initializeMillisTimer() {
  // Timer0 für millis() Funktion initialisieren (Genauer mit CTC Mode und Compare Match)
  TCCR0A = (1 << WGM01); // CTC Mode (Clear Timer on Compare Match)
  TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64 (für 1MHz CPU, ergibt ca. 1ms Interrupt mit OCR0A=15) - leicht angepasst für 1MHz
  OCR0A = 15; // Compare Value für ca. 1ms Interrupt bei 1MHz und Prescaler 64 (1000000 / 64 / 1000 - 1 = 14.625 -> 15)
  TIMSK0 = (1 << OCIE0A); // Timer0 Compare Match A Interrupt Enable
  timer0_millis = 0;
}


int main(void) {
    initializeIO();
    initializeTimekeepingTimer(); // Für die normale Uhrzeit
    initializePWM();
    initializeInterrupts();
    initializeMillisTimer(); // Millis Timer initialisieren (für DCF77 Timing)

    sei();

    lastDcf77SyncTime = 0; // Initialisierung der letzten Sync-Zeit

    while (1) {
        asm("nop");
        if (currentBrightnessLevelIndex == 3) {
            enterSleepMode();
        } else {
            enterSleepMode();
        }
        _delay_ms(5);

        // Regelmäßige DCF77-Synchronisation versuchen (z.B. jede Stunde)
        if (millis() - lastDcf77SyncTime >= DCF77_SYNC_INTERVAL_MS) {
            if (!dcf77Receiving) { // Starte DCF77 Empfang nur, wenn nicht schon aktiv
                dcf77Receiving = true;
                dcf77TimeoutTimer = 0;
                dcf77BitIndex = 0;
                dcf77SignalReceived = false;
                dcf77SyncSuccessful = false;
                // Optional: Sync-Versuch LED anzeigen
            }
            // decodeDcf77Signal() wird bereits im PCINT2_vect ISR aufgerufen, daher hier nicht erneut aufrufen.
            // Die Ergebnisse werden über dcf77SyncSuccessful und lastDcf77SyncTime Variablen verfügbar gemacht.

            if (dcf77SyncSuccessful) {
                // Optional: Erfolgreiche Sync-Anzeige (z.B. LED für kurze Zeit einschalten)
                // lastDcf77SyncTime wird in parseDcf77Data() aktualisiert
            } else if (dcf77Receiving == false && dcf77SyncSuccessful == false) {
                // Optional: Fehlgeschlagene Sync-Anzeige (z.B. Fehler-LED)
                // Kein neuer Sync-Versuch in dieser Schleifeniteration, da dcf77Receiving == false
            }
        }
    }
    return 0;
}