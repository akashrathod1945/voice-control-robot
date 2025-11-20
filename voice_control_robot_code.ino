/* Robust bluetooth + voice + obstacle robot
   Toggle ultrasonic usage by changing USE_ULTRASONIC to 0 or 1

   - SoftwareSerial on pins 10(RX),11(TX) for HC-05
   - AFMotor motors M1..M4
   - HC-SR04 ultrasonic (Trig=A1, Echo=A0) if enabled
   - Servo on pin 9 (change if needed)
*/

#include <Servo.h>
#include <AFMotor.h>
#include <SoftwareSerial.h>

#define USE_ULTRASONIC 0    // set 0 = no ultrasonic checks, 1 = enable HC-SR04 safety scan

#define Echo A0
#define Trig A1
#define SERVO_PIN 9

#define SPEED 170
#define SP_POINT 103
#define STOP_DISTANCE_CM 12
#define DANGEROUS_DISTANCE_CM 3
#define TURN_MS 400         // duration for left/right turn

SoftwareSerial BT(10, 11); // RX, TX (HC-05 TX -> Arduino 10, HC-05 RX <- Arduino 11 via level-shifter)

// Motors
AF_DCMotor M1(1);
AF_DCMotor M2(2);
AF_DCMotor M3(3);
AF_DCMotor M4(4);

// Servo
Servo servo;

// BT input buffer
String btBuf = "";
unsigned long lastBtRecv = 0;
const unsigned long BT_IDLE_MS = 80; // process partial buffer after this idle time

void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  Serial.println();
  Serial.println("Boot: Serial & BT at 9600");
  Serial.print("USE_ULTRASONIC = "); Serial.println(USE_ULTRASONIC);

  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);

  servo.attach(SERVO_PIN);
  servo.write(SP_POINT);

  M1.setSpeed(SPEED);
  M2.setSpeed(SPEED);
  M3.setSpeed(SPEED);
  M4.setSpeed(SPEED);

  stopAll();
  Serial.println("setup done");
}

void loop() {
  readBT();

  if (btBuf.length() > 0 && (millis() - lastBtRecv) > BT_IDLE_MS) {
    String raw = btBuf;
    raw.trim();
    if (raw.length() > 0) {
      Serial.print("BT raw: '"); Serial.print(raw); Serial.println("'");
      printBytes(raw);
      processCommand(raw);
    }
    btBuf = "";
  }
}

// ----- read bytes from HC-05 and append to btBuf -----
// process immediately on newline
void readBT() {
  while (BT.available()) {
    char c = (char)BT.read();
    // debug: show received byte
    Serial.print("[recv] ");
    if ((uint8_t)c >= 32 && (uint8_t)c <= 126) Serial.write(c); else Serial.print("(" + String((int)(uint8_t)c) + ")");
    Serial.print("  ("); Serial.print((int)(uint8_t)c); Serial.println(")");
    btBuf += c;
    lastBtRecv = millis();
    if (c == '\n' || c == '\r') {
      String toProc = btBuf;
      toProc.trim();
      if (toProc.length() > 0) {
        Serial.print("Newline: processing -> '"); Serial.print(toProc); Serial.println("'");
        printBytes(toProc);
        processCommand(toProc);
      } else {
        Serial.println("Newline but buffer empty after trim");
      }
      btBuf = "";
    }
  }
}

// ----- process the incoming string into a command -----
void processCommand(String raw) {
  raw.trim();
  if (raw.length() == 0) {
    Serial.println("Empty command (after trim)");
    return;
  }

  char first = raw.charAt(0);
  Serial.print("Processing cmd: '"); Serial.print(raw); Serial.print("'  first=");
  Serial.print(first); Serial.print("  ascii="); Serial.println((int)first);

  String cmd = raw;
  cmd.toLowerCase();

  if (cmd == "f" || cmd == "forward" || cmd == "^") {
    forward();
    BT.println("ACK:FORWARD");
  } else if (cmd == "b" || cmd == "back" || cmd == "backward" || cmd == "-") {
    backward();
    BT.println("ACK:BACK");
  } else if (cmd == "l" || cmd == "left" || cmd == "<") {
    handleLeftCommand();
    BT.println("ACK:LEFT");
  } else if (cmd == "r" || cmd == "right" || cmd == ">") {
    handleRightCommand();
    BT.println("ACK:RIGHT");
  } else if (cmd == "s" || cmd == "stop" || cmd == "*") {
    stopAll();
    BT.println("ACK:STOP");
  } else if (cmd == "auto" || cmd == "a") {
    // simple auto: forward until obstacle, allow BT override
    while (true) {
      int d = ultrasonic();
      if (d > 0 && d <= STOP_DISTANCE_CM) {
        stopAll();
        BT.println("AUTO_STOP");
        break;
      }
      forward();
      delay(50);
      if (BT.available()) {
        btBuf = "";
        Serial.println("BT override: leaving AUTO");
        break;
      }
    }
  } else {
    // try to find keywords inside the string
    if (cmd.indexOf("forward") >= 0) { forward(); BT.println("ACK:FORWARD"); }
    else if (cmd.indexOf("back") >= 0) { backward(); BT.println("ACK:BACK"); }
    else if (cmd.indexOf("left") >= 0) { handleLeftCommand(); BT.println("ACK:LEFT"); }
    else if (cmd.indexOf("right") >= 0) { handleRightCommand(); BT.println("ACK:RIGHT"); }
    else if (cmd.indexOf("stop") >= 0) { stopAll(); BT.println("ACK:STOP"); }
    else {
      Serial.println("Unrecognized command: " + raw);
      BT.print("UNKNOWN:"); BT.println(raw);
    }
  }
}

// ----- Left/Right handlers (with and without ultrasonic) -----
void handleLeftCommand() {
#if USE_ULTRASONIC
  int d = leftsee();        // moves servo and returns distance
  servo.write(SP_POINT);
  Serial.print("Leftscan returned: "); Serial.println(d);
  // Skip turn only if something dangerously close (<= DANGEROUS_DISTANCE_CM)
  if (d > 0 && d <= DANGEROUS_DISTANCE_CM) {
    Serial.println("Object too close on left — not turning");
    stopAll();
    return;
  }
#endif
  left();
  Serial.println("Performed LEFT turn");
  delay(TURN_MS);
  stopAll();
}

void handleRightCommand() {
#if USE_ULTRASONIC
  int d = rightsee();
  servo.write(SP_POINT);
  Serial.print("Rightscan returned: "); Serial.println(d);
  if (d > 0 && d <= DANGEROUS_DISTANCE_CM) {
    Serial.println("Object too close on right — not turning");
    stopAll();
    return;
  }
#endif
  right();
  Serial.println("Performed RIGHT turn");
  delay(TURN_MS);
  stopAll();
}

// ----- helpers -----
int ultrasonic() {
#if USE_ULTRASONIC
  digitalWrite(Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  long t = pulseIn(Echo, HIGH, 30000); // timeout 30ms
  if (t == 0) return -1;
  long cm = t / 29 / 2;
  return (int)cm;
#else
  return -1; // ultrasonic not used
#endif
}

void forward() {
  M1.run(FORWARD); M2.run(FORWARD); M3.run(FORWARD); M4.run(FORWARD);
  Serial.println("Motors: FORWARD");
}
void backward() {
  M1.run(BACKWARD); M2.run(BACKWARD); M3.run(BACKWARD); M4.run(BACKWARD);
  Serial.println("Motors: BACKWARD");
}
void right() {
  M1.run(BACKWARD); M2.run(BACKWARD); M3.run(FORWARD); M4.run(FORWARD);
  Serial.println("Motors: RIGHT");
}
void left() {
  M1.run(FORWARD); M2.run(FORWARD); M3.run(BACKWARD); M4.run(BACKWARD);
  Serial.println("Motors: LEFT");
}
void stopAll() {
  M1.run(RELEASE); M2.run(RELEASE); M3.run(RELEASE); M4.run(RELEASE);
  Serial.println("Motors: STOP");
}

int rightsee() {
#if USE_ULTRASONIC
  servo.write(20);
  delay(400); // give servo time to move
  int d = ultrasonic();
  Serial.print("Right scan: "); Serial.println(d);
  return d;
#else
  return -1;
#endif
}

int leftsee() {
#if USE_ULTRASONIC
  servo.write(180);
  delay(400); // give servo time to move
  int d = ultrasonic();
  Serial.print("Left scan: "); Serial.println(d);
  return d;
#else
  return -1;
#endif
}

void printBytes(String s) {
  Serial.print("Bytes: ");
  for (unsigned int i = 0; i < s.length(); ++i) {
    byte b = s.charAt(i);
    Serial.print((int)b);
    Serial.print("(");
    if (b < 16) Serial.print("0");
    Serial.print(String(b, HEX));
    Serial.print(") ");
  }
  Serial.println();
  Serial.println("------------------------");
}