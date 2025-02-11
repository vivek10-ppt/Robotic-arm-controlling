#include <AccelStepper.h>

// Define steppers and the pins they will use
AccelStepper stepper1(AccelStepper::DRIVER, 2, 3);
AccelStepper stepper2(AccelStepper::DRIVER, 4, 5);
AccelStepper stepper3(AccelStepper::DRIVER, 6, 7);

String receivedData = "";
bool newData = false;

void setup() {
  Serial.begin(115200); // Faster communication

  while (!Serial) {
    ; // Wait for serial port to connect
  }
  Serial.println("Arduino is ready");

  float acceleration = 50; // Set acceleration in steps per second per second
  stepper1.setMaxSpeed(1000); // Set max speed in steps per second
  stepper1.setAcceleration(acceleration); // Set acceleration
  stepper2.setMaxSpeed(1000); // Set max speed in steps per second
  stepper2.setAcceleration(acceleration); // Set acceleration
  stepper3.setMaxSpeed(1000); // Set max speed in steps per second
  stepper3.setAcceleration(acceleration); // Set acceleration
}

void loop() {
  receiveData();
  if (newData) {
    long steps1, steps2, steps3;
    parseData(receivedData, steps1, steps2, steps3);
    Serial.print("Received steps: ");
    Serial.print(steps1);
    Serial.print(", ");
    Serial.print(steps2);
    Serial.print(", ");
    Serial.println(steps3);

    stepper1.moveTo(steps1); // Move stepper1 to specified steps
    stepper2.moveTo(steps2); // Move stepper2 to specified steps
    stepper3.moveTo(steps3); // Move stepper3 to specified steps
    newData = false; // Reset newData flag after processing
  }

  stepper1.run();
  stepper2.run();
  stepper3.run();
}

void receiveData() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;
  const byte numChars = 32;
  char receivedChars[numChars];

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        receivedData = String(receivedChars);
        newData = true;

        // Print the entire received string for debugging
        Serial.print("Received data: ");
        Serial.println(receivedData);
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void parseData(String data, long &steps1, long &steps2, long &steps3) {
    int commaIndex1 = data.indexOf(',');
    int commaIndex2 = data.indexOf(',', commaIndex1 + 1);

    if (commaIndex1 == -1 || commaIndex2 == -1) {
        Serial.println("Error: Malformed data");
        steps1 = steps2 = steps3 = 0; // Default safe values
        return;
    }

    steps1 = data.substring(0, commaIndex1).toInt();
    steps2 = data.substring(commaIndex1 + 1, commaIndex2).toInt();
    steps3 = data.substring(commaIndex2 + 1).toInt();
}
