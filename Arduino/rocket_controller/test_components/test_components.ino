
// SERVO MOTORS CONTROL
/*
#include <Servo.h>

Servo myServo;   // Create a Servo object
int servoPin = 10; // Servo control pin
int position = 90; // Initial servo position (90 degrees)

void setup() {
  myServo.attach(servoPin); // Attach servo to the specified pin
  myServo.write(position);  // Set servo to the initial position

  Serial.begin(9600); // Start serial communication
  Serial.println("Servo Control via Keyboard");
  Serial.println("Press 'a' to decrease angle, 'd' to increase angle, or 'r' to reset to 90 degrees.");
}

void loop() {
  if (Serial.available()) {          // Check if data is available on the serial port
    char input = Serial.read();      // Read the input character

    if (input == 'a') {             // Decrease servo angle
      position -= 1;
      if (position < 0) position = 0;
      myServo.write(position);
      Serial.print("Position: ");
      Serial.println(position);
    } 
    else if (input == 'd') {        // Increase servo angle
      position += 1;
      if (position > 180) position = 180;
      myServo.write(position);
      Serial.print("Position: ");
      Serial.println(position);
    } 
    else if (input == 'r') {        // Reset servo to 90 degrees
      position = 90;
      myServo.write(position);
      Serial.println("Position reset to 90");
    }
    else {
      Serial.println("Invalid input. Use 'a', 'd', or 'r'.");
    }
  }
}
*/


/*
// BRUSHLESS MOTOR CONTROL
// NOTE: you have to use the option "No Line Editing" from the console, otherwise the motor will start rotating for an instant and then it will stop again.
#include <Servo.h>

Servo esc; // Create a servo object to control the ESC
int escPin = 11; // Connect the ESC signal wire to pin 11

void setup() {
  esc.attach(escPin,1000,2000); // Attach the ESC to the pin
  Serial.begin(9600); // Start the serial communication
  Serial.println("ESC initialized. Use values 0-180 for throttle.");

  // Arm the ESC
  Serial.println("Arming ESC...");
  esc.write(0); // Send a 0% throttle signal
  delay(2000);  // Wait 2 seconds
  Serial.println("ESC armed. Ready to control!");
}

void loop() {
  if (Serial.available() > 0) {
    int throttle = Serial.parseInt(); // Read input value
    if (throttle >= 0 && throttle <= 180) {
      esc.write(throttle); // Send throttle value to ESC
      Serial.print("Throttle set to: ");
      Serial.println(throttle);
    } else {
      Serial.println("Invalid input. Enter a value between 0 and 180.");
    }
  }
}

*/

/*
// RC RECEIVER
int throttlePin = 7; // Pin where the throttle signal wire is connected
unsigned long throttleValue; // Variable to store pulse width

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud
  pinMode(throttlePin, INPUT); // Set pin as input
  Serial.println("Reading RC Receiver Throttle Signal...");
}

void loop() {
  // Measure the duration of the pulse (in microseconds)
  throttleValue = pulseIn(throttlePin, HIGH, 25000); // Timeout of 25ms
  
  if (throttleValue > 900 && throttleValue < 2100) { // Valid pulse range
    Serial.print("Throttle Pulse Width: ");
    Serial.print(throttleValue);
    Serial.println(" µs");
  } else {
    Serial.println("No valid signal detected.");
  }

  delay(100); // Short delay to stabilize readings
}
*/






// RC CONTROLLER FOR THE ESC CONTROL
#include <Servo.h>

int receiverPin = 5;        // Throttle signal input from RC receiver
int escPin = 4;             // Signal output to ESC
unsigned long throttleSignal;       // Pulse width from receiver
unsigned long previousThrottle = 1000; // Last valid throttle signal
unsigned long lastSignalTime = 0;   // Time of last valid signal
unsigned long signalTimeout = 500;  // Signal loss timeout in milliseconds

Servo esc; // Create Servo object to control the ESC

void setup() {
  Serial.begin(9600); // Start serial communication
  pinMode(receiverPin, INPUT); // Receiver pin as input
  esc.attach(escPin);          // ESC signal pin
  esc.writeMicroseconds(1000); // Initialize ESC with minimum throttle

  Serial.println("Reading throttle and controlling ESC...");
  delay(2000); // Wait 2 seconds to ensure ESC arms properly
}

void loop() {
  // Read pulse width from receiver with 25 ms timeout
  throttleSignal = pulseIn(receiverPin, HIGH, 25000);

  // Check if signal is valid
  if (throttleSignal > 900 && throttleSignal < 2100) {
    previousThrottle = throttleSignal; // Update last valid throttle
    lastSignalTime = millis();         // Update last signal time
    Serial.print("Throttle Signal: ");
    Serial.print(throttleSignal);
    Serial.println(" µs");
  } else {
    // If signal is invalid, check timeout
    if (millis() - lastSignalTime > signalTimeout) {
      Serial.println("Signal lost for >0.5s. ESC shut down.");
      previousThrottle = 1000; // Set throttle to minimum (shut down ESC)
    } else {
      Serial.println("Signal lost. Holding last throttle value.");
    }
  }

  // Send last valid or shutdown signal to ESC
  esc.writeMicroseconds(previousThrottle);

  delay(20); // Small delay for signal smoothness
}




// RC CONTROLLER FOR THE ESC CONTROL AND THE SERVOS
// V1 - NO INTERRUPTS
/*
#include <Servo.h>

// ESC and throttle signal
int receiverPin = 7;        // Throttle signal input from RC receiver
int escPin = 11;             // Signal output to ESC
unsigned long throttleSignal;       // Pulse width from receiver
unsigned long previousThrottle = 1000; // Last valid throttle signal
unsigned long lastSignalTime = 0;   // Time of last valid signal
unsigned long signalTimeout = 500;  // Signal loss timeout in milliseconds

// Pitch and Roll
int pitchPin = 5;            // Pitch signal input (RC channel)
int rollPin = 6;             // Roll signal input (RC channel)
int pitchServoPin = 9;       // Servo pin for pitch control
int rollServoPin = 10;       // Servo pin for roll control

// Servo objects
Servo esc;          // ESC control
Servo pitchServo;   // Pitch servo control
Servo rollServo;    // Roll servo control

// Limits for pitch and roll servos
int pitchMax = 130;           // Range for the servo movement - real limit: 133
int pitchMin = 50;            // real limit: 45
int rollMax = 133;            // Range for the servo movement - real limit: 133
int rollMin = 50;             // real limit: 50

unsigned long pitchSignal;   // Pulse width for pitch
unsigned long rollSignal;    // Pulse width for roll

void setup() {
  Serial.begin(9600); // Start serial communication

  // Throttle setup 
  pinMode(receiverPin, INPUT); // Receiver pin as input
  esc.attach(escPin);          // ESC signal pin
  esc.writeMicroseconds(1000); // Initialize ESC with minimum throttle

  // Pitch and roll setup
  pinMode(pitchPin, INPUT);    // Pitch input pin
  pinMode(rollPin, INPUT);     // Roll input pin
  pitchServo.attach(pitchServoPin); // Attach pitch servo
  rollServo.attach(rollServoPin);   // Attach roll servo

  // Initialize servos at neutral position
  pitchServo.write(90); // Midpoint
  rollServo.write(90);    // Midpoint

  Serial.println("ESC and servo control initialized...");
  delay(2000); // Wait 2 seconds to ensure ESC arms properly
}

void loop() {
  // Read throttle signal - pulse width from receiver with 25 ms timeout
  throttleSignal = pulseIn(receiverPin, HIGH, 4000);

  // Process throttle
  // Check if signal is valid
  Serial.print("Throttle Signal: ");
  if (throttleSignal > 900 && throttleSignal < 2100) {
    previousThrottle = throttleSignal; // Update last valid throttle
    lastSignalTime = millis();         // Update last signal time
    Serial.print(throttleSignal);
    Serial.print(" µs");
  } else {
    // If signal is invalid, check timeout
    if (millis() - lastSignalTime > signalTimeout) {
      Serial.print("Lost for >0.5s"); // If lost for >0.5s, ESC shut down.
      previousThrottle = 1000; // Set throttle to minimum (shut down ESC)
    } else {
      Serial.print("Lost. Holding."); // If signal lost, hold last throttle value
    }
  }

  // Send last valid or shutdown signal to ESC
  esc.writeMicroseconds(previousThrottle);


  // Read and map pitch signal
  pitchSignal = pulseIn(pitchPin, HIGH, 9000);
  Serial.print("\t | Pitch: ");
  if (pitchSignal >= 1000 && pitchSignal <= 2000) {
    int pitchPosition = map(pitchSignal, 1000, 2000, 0, 180);
    pitchPosition = constrain(pitchPosition, pitchMin, pitchMax);
    pitchServo.write(pitchPosition);
    Serial.print(pitchPosition);
  } else {
    Serial.print("OOR-"); // Out of range
    Serial.print(pitchSignal);
  }


  // Read and map roll signal
  rollSignal = pulseIn(rollPin, HIGH, 9000);
  Serial.print("\t | Roll: ");
  if (rollSignal >= 1000 && rollSignal <= 2000) {
    int rollPosition = map(rollSignal, 1000, 2000, 0, 180);
    rollPosition = constrain(rollPosition, rollMin, rollMax);
    rollServo.write(rollPosition);
    Serial.print(rollPosition);
  } else {
    Serial.print("OOR-"); // Out of range
    Serial.print(rollSignal);
  }
  Serial.println();

  delay(20); // Small delay for signal smoothness

  
}
*/