/*
 * =========================================================================
 * Brushed DC Motor Control via ESC
 * * HOW IT WORKS: 
 * This code uses the <Servo.h> library to generate a Pulse Width Modulation 
 * (PWM) signal. A Brushed ESC interprets the "width" of a pulse (in 
 * microseconds) to decide the speed and direction of the motor:
 * - 1000µs: Full Power Reverse
 * - 1500µs: Neutral / Deadzone (Motor stops)
 * - 2000µs: Full Power Forward
 * * LIBRARIES: 
 * <Servo.h> is used because it handles the precise background timing (50Hz 
 * frequency) required by Hobby ESCs, freeing up the CPU for other tasks.
 * Author-Trishit Ghatak-2026
 * =========================================================================
 */

#include <Servo.h>

// --- CONFIGURATION ---
const int ESC_PIN = 13;        // Signal wire to Digital Pin 13
const int STOP_SIGNAL = 1500;   // The "Middle" signal where the motor stays still
const int FORWARD_SPEED = 1750; // Moderate forward speed
const int REVERSE_SPEED = 1250; // Moderate reverse speed

Servo brushed_ESC; // Create a servo object to represent our ESC

void setup() {
  // 1. Attach the ESC object to the physical pin
  brushed_ESC.attach(ESC_PIN);

  // 2. THE ARMING SEQUENCE (Safety)
  // Most brushed ESCs won't start until they "see" a neutral signal.
  // This prevents the model from zooming away as soon as you plug it in.
  brushed_ESC.writeMicroseconds(STOP_SIGNAL); 
  delay(2000); // Wait for the ESC to initialize/beep
}

void loop() {
  // --- PHASE 1: FORWARD ---
  // Tell the ESC to spin the motor forward at a set speed
  brushed_ESC.writeMicroseconds(FORWARD_SPEED);
  delay(3000); // Run for 3 seconds

  // --- PHASE 2: BRAKE/NEUTRAL ---
  // Bring the pulse back to 1500µs to stop the motor
  brushed_ESC.writeMicroseconds(STOP_SIGNAL);
  delay(1000); // Pause for 1 second

  // --- PHASE 3: REVERSE ---
  // A pulse lower than 1500µs tells the ESC to flip polarity and spin backward
  brushed_ESC.writeMicroseconds(REVERSE_SPEED);
  delay(3000); // Run for 3 seconds

  // --- PHASE 4: BRAKE/NEUTRAL ---
  brushed_ESC.writeMicroseconds(STOP_SIGNAL);
  delay(1000);
}