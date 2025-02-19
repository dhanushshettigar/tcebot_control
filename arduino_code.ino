#define LEFT_EN 9       // PWM pin for left motor
#define RIGHT_EN 10      // PWM pin for right motor
#define LEFT_FORWARD 2  // Direction pin for left motor forward
#define LEFT_BACKWARD 3 // Direction pin for left motor backward
#define RIGHT_FORWARD 4 // Direction pin for right motor forward
#define RIGHT_BACKWARD 5 // Direction pin for right motor backward

int lPWM = 0;
int rPWM = 0;
bool lDIR = true;
bool rDIR = true;
String inputString = ""; 

void setup() {
    Serial.begin(9600);
    pinMode(LEFT_EN, OUTPUT);
    pinMode(RIGHT_EN, OUTPUT);
    pinMode(LEFT_FORWARD, OUTPUT);
    pinMode(LEFT_BACKWARD, OUTPUT);
    pinMode(RIGHT_FORWARD, OUTPUT);
    pinMode(RIGHT_BACKWARD, OUTPUT);

    digitalWrite(LEFT_FORWARD, LOW);
    digitalWrite(LEFT_BACKWARD, LOW);
    digitalWrite(RIGHT_FORWARD, LOW);
    digitalWrite(RIGHT_BACKWARD, LOW);
}

void loop() {
    if (Serial.available()) {
        char incomingChar = Serial.read();
        
        if (incomingChar == '\n') {
            processCommand(inputString);
            inputString = "";
        } else {
            inputString += incomingChar;
        }
    }
}

void processCommand(String command) {
    int values[4]; 
    int index = 0;
    char *token = strtok(&command[0], ",");

    while (token != NULL && index < 4) {
        values[index++] = atoi(token);
        token = strtok(NULL, ",");
    }

    if (index == 4) {
        lDIR = values[0];
        lPWM = values[1];
        rDIR = values[2];
        rPWM = values[3];

        controlMotors();
    }
}

void controlMotors() {
    // Left motor direction
    digitalWrite(LEFT_FORWARD, lDIR);
    digitalWrite(LEFT_BACKWARD, !lDIR);

    // Right motor direction
    digitalWrite(RIGHT_FORWARD, rDIR);
    digitalWrite(RIGHT_BACKWARD, !rDIR);

    // Set PWM speed
    analogWrite(LEFT_EN, lPWM);
    analogWrite(RIGHT_EN, rPWM);
    
    Serial.print("L_DIR: "); Serial.print(lDIR);
    Serial.print(" L_PWM: "); Serial.print(lPWM);
    Serial.print(" R_DIR: "); Serial.print(rDIR);
    Serial.print(" R_PWM: "); Serial.println(rPWM);
}
