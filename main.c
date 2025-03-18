/*
Binary Clock by Ben Kaden
Enhanced and Refactored Version
Functionality: Binary clock with adjustable brightness and sleep mode.
*/

/*
Pin Configuration:
- PWM outputs (brightness control): PB1, PB2
- Brightness/Sleep button: PD7
- Hours button: PD6
- Minutes button: PB0
*/

/*
Compilation Instructions (AVR-GCC & avrdude example):
1. avr-gcc -mmcu=atmega48 -DF_CPU=1000000UL -Wcpp -Os -o binary_clock.elf binary_clock.c
2. avr-objcopy -O ihex -R .eeprom binary_clock.elf binary_clock.hex
3. avrdude -c usbasp -p m48 -U flash:w:binary_clock.hex:i
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>

// Timekeeping variables (volatile for ISR access)
volatile uint8_t currentSeconds = 50;       // Initial seconds for testing (adjust to 0 for normal operation)
volatile uint8_t currentMinutes = 30;
volatile uint8_t currentHours = 12;

// Brightness control variables
volatile uint8_t currentBrightnessLevelIndex = 0; // Index for brightness levels array
volatile uint8_t brightnessButtonHoldTimer = 0;   // Timer to detect long press for brightness button
volatile bool isBrightnessButtonHeld = false;     // Flag to indicate if brightness button is held

// Minute adjust variables
volatile uint8_t minutesButtonHoldTimer = 0;      // Timer to detect long press for minutes button
volatile bool isMinutesButtonHeld = false;        // Flag to indicate if minutes button is held

// Hour adjust variables
volatile uint8_t hoursButtonHoldTimer = 0;        // Timer to detect long press for hours button
volatile bool isHoursButtonHeld = false;          // Flag to indicate if hours button is held

// Define brightness levels (reversed for OCR values, 0 = off, 255 = max brightness)
const uint8_t brightnessLevels[] = {254, 248, 156, 0};

// Function Prototypes
void initializeTimekeepingTimer(void);
void initializePWM(void);
void initializeIO(void);
void initializeInterrupts(void);
void updateBinaryDisplay(void);
void enterSleepMode(void);

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
    DDRD &= ~((1 << PD6) | (1 << PD7)); // PD6 (Hours button), PD7 (Brightness/Sleep button) as inputs
    // Pull-up resistors are already enabled for PORTB and PORTD
}

// Initialize external interrupts for button presses (using Pin Change Interrupts)
void initializeInterrupts(void) {
    // Enable Pin Change Interrupts for PORTB and PORTD groups
    PCICR |= (1 << PCIE0) | (1 << PCIE2); // Enable PCIE0 for PORTB, PCIE2 for PORTD

    // Specify which pins trigger interrupts within each port group
    PCMSK0 |= (1 << PCINT0);              // PB0 (Minutes button) triggers interrupt
    PCMSK2 |= (1 << PCINT22) | (1 << PCINT23); // PD6 (Hours button), PD7 (Brightness/Sleep button) trigger interrupts
}

// Update the binary LED display to show current hours and minutes
void updateBinaryDisplay(void) {
    // Display hours on PORTC (PC0-PC4) -  using bitwise AND to clear previous hour value and then ORing with new hour value
    PORTC = (PORTC & ~0x1F) | (currentHours & 0x1F); // Keep other bits of PORTC unchanged

    // Display minutes on PORTD (PD0-PD5) - using bitwise AND to clear previous minute value and then ORing with new minute value
    PORTD = (PORTD & ~0x3F) | (currentMinutes & 0x3F); // Keep other bits of PORTD unchanged
}

// Timer2 Overflow Interrupt Service Routine: handles timekeeping and button hold timers
ISR(TIMER2_OVF_vect) {
    // Increment seconds counter
    currentSeconds++;

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

    // Time update logic: increment minutes, hours, and reset seconds as needed
    if (currentSeconds >= 60) {
        currentSeconds = 0;
        currentMinutes++;
        if (currentMinutes >= 60) {
            currentMinutes = 0;
            currentHours++;
            if (currentHours >= 24) {
                currentHours = 0; // Reset hours to 0 after 24 hours (24-hour format)
            }
        }
    }

    updateBinaryDisplay(); // Refresh the LED display with updated time
}

// Pin Change Interrupt Service Routine for PORTB (handles minutes button - PB0)
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

// Pin Change Interrupt Service Routine for PORTD (handles hours and brightness/sleep buttons - PD6 and PD7)
ISR(PCINT2_vect) {
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

int main(void) {
    initializeIO();             // Configure I/O ports (LEDs, buttons, pull-ups)
    initializeTimekeepingTimer(); // Set up Timer2 for timekeeping
    initializePWM();            // Set up Timer1 for PWM brightness control
    initializeInterrupts();       // Configure pin change interrupts for buttons

    sei(); // Enable global interrupts - necessary for interrupts to be processed

    while (1) { // Main program loop - runs continuously
        asm("nop"); // No operation - can be used for debugging or as a placeholder

        if (currentBrightnessLevelIndex == 3) { // Enter sleep mode if brightness is off
            enterSleepMode(); // Enter sleep mode to save power when display is off
        } else {
            enterSleepMode(); // Enter idle sleep mode when display is on, for reduced power consumption
        }
        _delay_ms(5); // Small delay after wake-up if needed (e.g., for debouncing or settling)
    }

    return 0; // Standard return 0 for main function indicating successful execution (though this point is never reached in this infinite loop)
}/*
Binary Clock by Ben Kaden
Enhanced and Refactored Version
Functionality: Binary clock with adjustable brightness and sleep mode.
*/

/*
Pin Configuration:
- PWM outputs (brightness control): PB1, PB2
- Brightness/Sleep button: PD7
- Hours button: PD6
- Minutes button: PB0
*/

/*
Compilation Instructions (AVR-GCC & avrdude example):
1. avr-gcc -mmcu=atmega48 -DF_CPU=1000000UL -Wcpp -Os -o binary_clock.elf binary_clock.c
2. avr-objcopy -O ihex -R .eeprom binary_clock.elf binary_clock.hex
3. avrdude -c usbasp -p m48 -U flash:w:binary_clock.hex:i
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>

// Timekeeping variables (volatile for ISR access)
volatile uint8_t currentSeconds = 50;       // Initial seconds for testing (adjust to 0 for normal operation)
volatile uint8_t currentMinutes = 30;
volatile uint8_t currentHours = 12;

// Brightness control variables
volatile uint8_t currentBrightnessLevelIndex = 0; // Index for brightness levels array
volatile uint8_t brightnessButtonHoldTimer = 0;   // Timer to detect long press for brightness button
volatile bool isBrightnessButtonHeld = false;     // Flag to indicate if brightness button is held

// Minute adjust variables
volatile uint8_t minutesButtonHoldTimer = 0;      // Timer to detect long press for minutes button
volatile bool isMinutesButtonHeld = false;        // Flag to indicate if minutes button is held

// Hour adjust variables
volatile uint8_t hoursButtonHoldTimer = 0;        // Timer to detect long press for hours button
volatile bool isHoursButtonHeld = false;          // Flag to indicate if hours button is held

// Define brightness levels (reversed for OCR values, 0 = off, 255 = max brightness)
const uint8_t brightnessLevels[] = {254, 248, 156, 0};

// Function Prototypes
void initializeTimekeepingTimer(void);
void initializePWM(void);
void initializeIO(void);
void initializeInterrupts(void);
void updateBinaryDisplay(void);
void enterSleepMode(void);

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
    DDRD &= ~((1 << PD6) | (1 << PD7)); // PD6 (Hours button), PD7 (Brightness/Sleep button) as inputs
    // Pull-up resistors are already enabled for PORTB and PORTD
}

// Initialize external interrupts for button presses (using Pin Change Interrupts)
void initializeInterrupts(void) {
    // Enable Pin Change Interrupts for PORTB and PORTD groups
    PCICR |= (1 << PCIE0) | (1 << PCIE2); // Enable PCIE0 for PORTB, PCIE2 for PORTD

    // Specify which pins trigger interrupts within each port group
    PCMSK0 |= (1 << PCINT0);              // PB0 (Minutes button) triggers interrupt
    PCMSK2 |= (1 << PCINT22) | (1 << PCINT23); // PD6 (Hours button), PD7 (Brightness/Sleep button) trigger interrupts
}

// Update the binary LED display to show current hours and minutes
void updateBinaryDisplay(void) {
    // Display hours on PORTC (PC0-PC4) -  using bitwise AND to clear previous hour value and then ORing with new hour value
    PORTC = (PORTC & ~0x1F) | (currentHours & 0x1F); // Keep other bits of PORTC unchanged

    // Display minutes on PORTD (PD0-PD5) - using bitwise AND to clear previous minute value and then ORing with new minute value
    PORTD = (PORTD & ~0x3F) | (currentMinutes & 0x3F); // Keep other bits of PORTD unchanged
}

// Timer2 Overflow Interrupt Service Routine: handles timekeeping and button hold timers
ISR(TIMER2_OVF_vect) {
    // Increment seconds counter
    currentSeconds++;

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

    // Time update logic: increment minutes, hours, and reset seconds as needed
    if (currentSeconds >= 60) {
        currentSeconds = 0;
        currentMinutes++;
        if (currentMinutes >= 60) {
            currentMinutes = 0;
            currentHours++;
            if (currentHours >= 24) {
                currentHours = 0; // Reset hours to 0 after 24 hours (24-hour format)
            }
        }
    }

    updateBinaryDisplay(); // Refresh the LED display with updated time
}

// Pin Change Interrupt Service Routine for PORTB (handles minutes button - PB0)
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

// Pin Change Interrupt Service Routine for PORTD (handles hours and brightness/sleep buttons - PD6 and PD7)
ISR(PCINT2_vect) {
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

int main(void) {
    initializeIO();             // Configure I/O ports (LEDs, buttons, pull-ups)
    initializeTimekeepingTimer(); // Set up Timer2 for timekeeping
    initializePWM();            // Set up Timer1 for PWM brightness control
    initializeInterrupts();       // Configure pin change interrupts for buttons

    sei(); // Enable global interrupts - necessary for interrupts to be processed

    while (1) { // Main program loop - runs continuously
        asm("nop"); //*
Binary Clock by Ben Kaden
Enhanced and Refactored Version
Functionality: Binary clock with adjustable brightness and sleep mode.
*/

/*
Pin Configuration:
- PWM outputs (brightness control): PB1, PB2
- Brightness/Sleep button: PD7
- Hours button: PD6
- Minutes button: PB0
*/

/*
Compilation Instructions (AVR-GCC & avrdude example):
1. avr-gcc -mmcu=atmega48 -DF_CPU=1000000UL -Wcpp -Os -o binary_clock.elf binary_clock.c
2. avr-objcopy -O ihex -R .eeprom binary_clock.elf binary_clock.hex
3. avrdude -c usbasp -p m48 -U flash:w:binary_clock.hex:i
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>

// Timekeeping variables (volatile for ISR access)
volatile uint8_t currentSeconds = 50;       // Initial seconds for testing (adjust to 0 for normal operation)
volatile uint8_t currentMinutes = 30;
volatile uint8_t currentHours = 12;

// Brightness control variables
volatile uint8_t currentBrightnessLevelIndex = 0; // Index for brightness levels array
volatile uint8_t brightnessButtonHoldTimer = 0;   // Timer to detect long press for brightness button
volatile bool isBrightnessButtonHeld = false;     // Flag to indicate if brightness button is held

// Minute adjust variables
volatile uint8_t minutesButtonHoldTimer = 0;      // Timer to detect long press for minutes button
volatile bool isMinutesButtonHeld = false;        // Flag to indicate if minutes button is held

// Hour adjust variables
volatile uint8_t hoursButtonHoldTimer = 0;        // Timer to detect long press for hours button
volatile bool isHoursButtonHeld = false;          // Flag to indicate if hours button is held

// Define brightness levels (reversed for OCR values, 0 = off, 255 = max brightness)
const uint8_t brightnessLevels[] = {254, 248, 156, 0};

// Function Prototypes
void initializeTimekeepingTimer(void);
void initializePWM(void);
void initializeIO(void);
void initializeInterrupts(void);
void updateBinaryDisplay(void);
void enterSleepMode(void);

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
    DDRD &= ~((1 << PD6) | (1 << PD7)); // PD6 (Hours button), PD7 (Brightness/Sleep button) as inputs
    // Pull-up resistors are already enabled for PORTB and PORTD
}

// Initialize external interrupts for button presses (using Pin Change Interrupts)
void initializeInterrupts(void) {
    // Enable Pin Change Interrupts for PORTB and PORTD groups
    PCICR |= (1 << PCIE0) | (1 << PCIE2); // Enable PCIE0 for PORTB, PCIE2 for PORTD

    // Specify which pins trigger interrupts within each port group
    PCMSK0 |= (1 << PCINT0);              // PB0 (Minutes button) triggers interrupt
    PCMSK2 |= (1 << PCINT22) | (1 << PCINT23); // PD6 (Hours button), PD7 (Brightness/Sleep button) trigger interrupts
}

// Update the binary LED display to show current hours and minutes
void updateBinaryDisplay(void) {
    // Display hours on PORTC (PC0-PC4) -  using bitwise AND to clear previous hour value and then ORing with new hour value
    PORTC = (PORTC & ~0x1F) | (currentHours & 0x1F); // Keep other bits of PORTC unchanged

    // Display minutes on PORTD (PD0-PD5) - using bitwise AND to clear previous minute value and then ORing with new minute value
    PORTD = (PORTD & ~0x3F) | (currentMinutes & 0x3F); // Keep other bits of PORTD unchanged
}

// Timer2 Overflow Interrupt Service Routine: handles timekeeping and button hold timers
ISR(TIMER2_OVF_vect) {
    // Increment seconds counter
    currentSeconds++;

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

    // Time update logic: increment minutes, hours, and reset seconds as needed
    if (currentSeconds >= 60) {
        currentSeconds = 0;
        currentMinutes++;
        if (currentMinutes >= 60) {
            currentMinutes = 0;
            currentHours++;
            if (currentHours >= 24) {
                currentHours = 0; // Reset hours to 0 after 24 hours (24-hour format)
            }
        }
    }

    updateBinaryDisplay(); // Refresh the LED display with updated time
}

// Pin Change Interrupt Service Routine for PORTB (handles minutes button - PB0)
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

// Pin Change Interrupt Service Routine for PORTD (handles hours and brightness/sleep buttons - PD6 and PD7)
ISR(PCINT2_vect) {
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

int main(void) {
    initializeIO();             // Configure I/O ports (LEDs, buttons, pull-ups)
    initializeTimekeepingTimer(); // Set up Timer2 for timekeeping
    initializePWM();            // Set up Timer1 for PWM brightness control
    initializeInterrupts();       // Configure pin change interrupts for buttons

    sei(); // Enable global interrupts - necessary for interrupts to be processed

    while (1) { // Main program loop - runs continuously
        asm("nop"); // No operation - can be used for debugging or as a placeholder
/*
Binary Clock by Ben Kaden
Enhanced and Refactored Version
Functionality: Binary clock with adjustable brightness and sleep mode.
*/

/*
Pin Configuration:
- PWM outputs (brightness control): PB1, PB2
- Brightness/Sleep button: PD7
- Hours button: PD6
- Minutes button: PB0
*/

/*
Compilation Instructions (AVR-GCC & avrdude example):
1. avr-gcc -mmcu=atmega48 -DF_CPU=1000000UL -Wcpp -Os -o binary_clock.elf binary_clock.c
2. avr-objcopy -O ihex -R .eeprom binary_clock.elf binary_clock.hex
3. avrdude -c usbasp -p m48 -U flash:w:binary_clock.hex:i
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>

// Timekeeping variables (volatile for ISR access)
volatile uint8_t currentSeconds = 50;       // Initial seconds for testing (adjust to 0 for normal operation)
volatile uint8_t currentMinutes = 30;
volatile uint8_t currentHours = 12;

// Brightness control variables
volatile uint8_t currentBrightnessLevelIndex = 0; // Index for brightness levels array
volatile uint8_t brightnessButtonHoldTimer = 0;   // Timer to detect long press for brightness button
volatile bool isBrightnessButtonHeld = false;     // Flag to indicate if brightness button is held

// Minute adjust variables
volatile uint8_t minutesButtonHoldTimer = 0;      // Timer to detect long press for minutes button
volatile bool isMinutesButtonHeld = false;        // Flag to indicate if minutes button is held

// Hour adjust variables
volatile uint8_t hoursButtonHoldTimer = 0;        // Timer to detect long press for hours button
volatile bool isHoursButtonHeld = false;          // Flag to indicate if hours button is held

// Define brightness levels (reversed for OCR values, 0 = off, 255 = max brightness)
const uint8_t brightnessLevels[] = {254, 248, 156, 0};

// Function Prototypes
void initializeTimekeepingTimer(void);
void initializePWM(void);
void initializeIO(void);
void initializeInterrupts(void);
void updateBinaryDisplay(void);
void enterSleepMode(void);

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
    DDRD &= ~((1 << PD6) | (1 << PD7)); // PD6 (Hours button), PD7 (Brightness/Sleep button) as inputs
    // Pull-up resistors are already enabled for PORTB and PORTD
}

// Initialize external interrupts for button presses (using Pin Change Interrupts)
void initializeInterrupts(void) {
    // Enable Pin Change Interrupts for PORTB and PORTD groups
    PCICR |= (1 << PCIE0) | (1 << PCIE2); // Enable PCIE0 for PORTB, PCIE2 for PORTD

    // Specify which pins trigger interrupts within each port group
    PCMSK0 |= (1 << PCINT0);              // PB0 (Minutes button) triggers interrupt
    PCMSK2 |= (1 << PCINT22) | (1 << PCINT23); // PD6 (Hours button), PD7 (Brightness/Sleep button) trigger interrupts
}

// Update the binary LED display to show current hours and minutes
void updateBinaryDisplay(void) {
    // Display hours on PORTC (PC0-PC4) -  using bitwise AND to clear previous hour value and then ORing with new hour value
    PORTC = (PORTC & ~0x1F) | (currentHours & 0x1F); // Keep other bits of PORTC unchanged

    // Display minutes on PORTD (PD0-PD5) - using bitwise AND to clear previous minute value and then ORing with new minute value
    PORTD = (PORTD & ~0x3F) | (currentMinutes & 0x3F); // Keep other bits of PORTD unchanged
}

// Timer2 Overflow Interrupt Service Routine: handles timekeeping and button hold timers
ISR(TIMER2_OVF_vect) {
    // Increment seconds counter
    currentSeconds++;

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

    // Time update logic: increment minutes, hours, and reset seconds as needed
    if (currentSeconds >= 60) {
        currentSeconds = 0;
        currentMinutes++;
        if (currentMinutes >= 60) {
            currentMinutes = 0;
            currentHours++;
            if (currentHours >= 24) {
                currentHours = 0; // Reset hours to 0 after 24 hours (24-hour format)
            }
        }
    }

    updateBinaryDisplay(); // Refresh the LED display with updated time
}

// Pin Change Interrupt Service Routine for PORTB (handles minutes button - PB0)
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

// Pin Change Interrupt Service Routine for PORTD (handles hours and brightness/sleep buttons - PD6 and PD7)
ISR(PCINT2_vect) {
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

int main(void) {
    initializeIO();             // Configure I/O ports (LEDs, buttons, pull-ups)
    initializeTimekeepingTimer(); // Set up Timer2 for timekeeping
    initializePWM();            // Set up Timer1 for PWM brightness control
    initializeInterrupts();       // Configure pin change interrupts for buttons

    sei(); // Enable global interrupts - necessary for interrupts to be processed

    while (1) { // Main program loop - runs continuously
        asm("nop"); // No operation - can be used for debugging or as a placeholder

        if (currentBrightnessLevelIndex == 3) { // Enter sleep mode if brightness is off
            enterSleepMode(); // Enter sleep mode to save power when display is off
        } else {
            enterSleepMode(); // Enter idle sleep mode when display is on, for reduced power consumption
        }
        _delay_ms(5); // Small delay after wake-up if needed (e.g., for debouncing or settling)
    }

    return 0; // Standard return 0 for main function indicating successful execution (though this point is never reached in this infinite loop)
}
        if (currentBrightnessLevelIndex == 3) { // Enter sleep mode if brightness is off
            enterSleepMode(); // Enter sleep mode to save power when display is off
        } else {
            enterSleepMode(); // Enter idle sleep mode when display is on, for reduced power consumption
        }
        _delay_ms(5); // Small delay after wake-up if needed (e.g., for debouncing or settling)
    }

    return 0; // Standard return 0 for main function indicating successful execution (though this point is never reached in this infinite loop)
}/ No operation - can be used for debugging or as a placeholder

        if (currentBrightnessLevelIndex == 3) { // Enter sleep mode if brightness is off
            enterSleepMode(); // Enter sleep mode to save power when display is off
        } else {
            enterSleepMode(); // Enter idle sleep mode when display is on, for reduced power consumption
        }
        _delay_ms(5); // Small delay after wake-up if needed (e.g., for debouncing or settling)
    }

    return 0; // Standard return 0 for main function indicating successful execution (though this point is never reached in this infinite loop)
}
