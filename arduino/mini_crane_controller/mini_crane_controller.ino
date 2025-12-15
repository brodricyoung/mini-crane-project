// Number of buttons
const uint8_t NUM_BUTTONS = 6;

// Pins for the buttons
uint8_t buttonPins[NUM_BUTTONS] = {2, 3, 4, 5, 6, 7};

// Track the previous button states so we only send stop once
bool prevState[NUM_BUTTONS];

const int buzzerPin = 8;
const int ledPin = 9;

// Serial input buffer
String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(115200);

  // Configure button pins as INPUT_PULLUP
  for (int i = 0; i < NUM_BUTTONS; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    prevState[i] = HIGH; // HIGH = not pressed
  }

  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
}

void loop() {

  // ----------------------------
  // 1. Handle incoming UART data
  // ----------------------------
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }

  // ----------------------------
  // 2. Handle button reading
  // ----------------------------
  for (int i = 0; i < NUM_BUTTONS; i++) {

    bool current = digitalRead(buttonPins[i]);

    // ACTIVE LOW → pressed = LOW
    if (current == LOW) {
      Serial.print("BTN");
      Serial.print(i);
      Serial.println("_PRESS");
    }

    // Detect release edge
    if (current == HIGH && prevState[i] == LOW) {
      Serial.print("BTN");
      Serial.print(i);
      Serial.println("_RELEASE");
    }

    prevState[i] = current;
  }

  delay(20); // Simple debounce + reduce spam
}


// ----------------------------------------------------
// UART receive interrupt-style buffer (non-blocking)
// ----------------------------------------------------
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}


// ----------------------------------------------------
// PROCESS COMMANDS FROM STM32
// ----------------------------------------------------
void processCommand(String cmd) {

  // Turn buzzer ON
  if (cmd == "BUZZ_ON") {
    tone(buzzerPin, 75);     // 75 Hz
    digitalWrite(ledPin, HIGH);
  }

  // Turn buzzer OFF
  else if (cmd == "BUZZ_OFF") {
    noTone(buzzerPin);
    digitalWrite(ledPin, LOW);
  }
}
