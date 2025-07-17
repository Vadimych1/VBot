#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define ABS(a) ((a > 0) ? (a) : (-a))

#define PWM_A 10
#define PWM_B 11

#define AA 8
#define AB 9
#define BA 12
#define BB 13

int accelerationSteps = 60;
int acceleration_time = 600;

LiquidCrystal_I2C lcd(0x27, 16, 2);

int prevLeftSpeed = 0;
int prevRightSpeed = 0;

int leftSpeed = 0;
int rightSpeed = 0;

bool accelerate = false;

double currentAccelerateStep = 0.0;
bool reaccelerate = false;

void setup() {
  lcd.init();
  lcd.backlight();

  pinMode(AA, OUTPUT);
  pinMode(AB, OUTPUT);
  pinMode(BA, OUTPUT);
  pinMode(BB, OUTPUT);

  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);

  digitalWrite(AA, HIGH);
  digitalWrite(AB, LOW);
  digitalWrite(BA, HIGH);
  digitalWrite(BB, LOW);

  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);

  Serial.begin(9600);
}

void loop() {
  // Serial bytes protocol
  //
  // lcd-backlight(0/1):smooth-acceleration(0/1):right-speed(0-255):right-sign(0/1):left-speed(0-255):left-sign(0/1);
  while (Serial.available()) { // while used if commands are stacked
    // Flags
    const bool enable = (Serial.read() == 1);
    if (enable) {
      lcd.backlight();
    } else {
      lcd.noBacklight();
    }

    accelerate = (Serial.read() == 1);
    
    // Speeds
    const int r = Serial.read();
    const int rSign = range(Serial.read() * 2 - 1, -1, 1); // 1 or -1
    const int l = Serial.read();
    const int lSign = range(Serial.read() * 2 - 1, -1, 1); // 1 or - 1

    if (r * rSign != rightSpeed) {
      prevRightSpeed = rightSpeed;
      rightSpeed = (int) (r * rSign);
      setRight(rightSpeed);
    }

    if (l * lSign != leftSpeed) {
      prevLeftSpeed = leftSpeed;
      leftSpeed = (int) (l * lSign);
      setLeft(leftSpeed);
    }

    acceleration_time = range(Serial.read() * 10, 100, 2500);
  }

  // Smoothly accelerate
  if (accelerate && reaccelerate) {
    const double target_a = (double) range(ABS(leftSpeed), 0, 255);
    const double target_b = (double) range(ABS(rightSpeed), 0, 255);

    const double prev_target_a = (double) range(ABS(prevLeftSpeed), 0, 255);
    const double prev_target_b = (double) range(ABS(prevRightSpeed), 0, 255);

    const double delta_a = target_a - prev_target_a;
    const double delta_b = target_b - prev_target_b;

    const int d = (int) (acceleration_time / accelerationSteps);

    while (currentAccelerateStep < accelerationSteps) {
      const double x = (currentAccelerateStep / accelerationSteps);
      const double val = 1 - (1 - x) * (1 - x);

      const int val_a = (int) (prev_target_a + delta_a * val);
      const int val_b = (int) (prev_target_b + delta_b * val);

      analogWrite(PWM_A, val_a);
      analogWrite(PWM_B, val_b);
      
      delay(d);

      currentAccelerateStep += 1.0;
    }

    reaccelerate = false;
  } else {
    delay(100);
  }

  // Display data
  updateScreen();
}

void updateScreen() {
  // Redraw data on the LCD display
  char leftSt[17];
  char rightSt[17];

  sprintf(leftSt, "L:%d|Acc:%dms", leftSpeed, acceleration_time);
  sprintf(rightSt, "R:%d|Acc:%s", rightSpeed, accelerate ? "ON" : "OFF");

  flushText();
  displayText(leftSt, 0);
  displayText(rightSt, 1);
}

// Displays a string on LCD display (first row, first col)
void displayText(const char* text) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.printstr(text);
}

// Displays a string on LCD display
//
// Allows to change row to write (first col)
void displayText(const char* text, int row) {
  lcd.setCursor(0, row);
  lcd.printstr(text);
}


// Displays a string on LCD display
//
// Allows to change row and col to write
void displayText(const char* text, int row, int col) {
  lcd.setCursor(col, row);
  lcd.printstr(text);
}

// Clears LCD display
//
// Useful if displayText with rows used
void flushText() {
  lcd.clear();
}

// Puts x in range of [min; max]
int range(int x, int min, int max) {
  return (x >= min && x <= max) ? x : ((x >= min) ? max : min);
}

// Changes speed of the left motor
//
// if speed is > 0, then spins motor CW, otherwise CCW
//
// if speed is 0, stops motor
void setLeft(int speed) {
  digitalWrite(AA, LOW);
  digitalWrite(AB, LOW);

  if (speed > 0) {
    digitalWrite(AA, HIGH);
  } else if (speed < 0) {
    digitalWrite(AB, HIGH);
  }

  if (accelerate && speed != 0) {
    analogWrite(PWM_A, 0);
    
    reaccelerate = true;
    currentAccelerateStep = 0;
  } else {
    analogWrite(PWM_A, range(ABS(speed), 0, 255));
  }
}

// Changes speed of the right motor
//
// if speed is > 0, then spins motor CCW, otherwise CW
//
// if speed is 0, stops motor
void setRight(int speed) {
  speed = -speed;

  digitalWrite(BA, LOW);
  digitalWrite(BB, LOW);

  if (speed > 0) {
    digitalWrite(BA, HIGH);
  } else if (speed < 0) {
    digitalWrite(BB, HIGH);
  }

  if (accelerate) {
    analogWrite(PWM_B, 0);
    
    reaccelerate = true;
    currentAccelerateStep = 0;
  } else {
    analogWrite(PWM_B, range(ABS(speed), 0, 255));
  }
}
