#include <Servo.h>
#include <LiquidCrystal_I2C.h>

#define X_SERVO_PIN_NUM     9
#define Y_SERVO_PIN_NUM     10
#define JOYSTICK_X_PIN_NUM  A0
#define JOYSTICK_Y_PIN_NUM  A1
#define START_INTERRUPT_PIN 2
#define END_INTERRUPT_PIN   3
#define RANKING_BUTTON_PIN  11

// Global Variables
Servo g_Xservo;
Servo g_Yservo;
LiquidCrystal_I2C lcd(0x27, 16, 2);



bool g_bDebugMode = true;



bool g_btimerControllerStarted = false;
unsigned long g_timerStart, g_timerEnd;
int set_maxAngle = 180;
int g_arrayNum = 0;
float g_ranking[100];

// end Global Variables

// Functions
void initializing();
int absF(int inputValue);
int getJoystickDataLocationX();
int getJoystickDataLocationY();
void moveLocationX(int angle);
void moveLocationY(int angle);
int getLocationXServoAngle();
int getLocationYServoAngle();
bool is_pressedSwitch();
void Debug();

void timerEndController() {
  if (g_btimerControllerStarted){
    g_btimerControllerStarted = false;
  }
}

void timerStartController() {
  if (!g_btimerControllerStarted) {
    g_btimerControllerStarted = true;
  }
  Serial.println(g_btimerControllerStarted);
}

// end Functions

void setup() {
  Serial.begin(9600);
  g_Xservo.attach(X_SERVO_PIN_NUM);
  g_Yservo.attach(Y_SERVO_PIN_NUM);
  lcd.init();
  lcd.backlight();
  pinMode(START_INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(END_INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(RANKING_BUTTON_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(START_INTERRUPT_PIN), timerStartController, HIGH);
  attachInterrupt(digitalPinToInterrupt(END_INTERRUPT_PIN), timerEndController, HIGH);
  lcd.print("Initializing...");
  initializing();
  lcd.clear();
  lcd.print("Starting...");
  delay(1000);
  lcd.clear();
  lcd.print("Ready!");
}

void loop() {
  if (g_bDebugMode) {
    Debug();
  }
  if (!g_btimerControllerStarted) {
    if (getJoystickDataLocationX() > 600 ||
        getJoystickDataLocationX() < 400 &&
        getJoystickDataLocationY() > 600 ||
        getJoystickDataLocationY() < 400) {

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Plz press switch");
      if (digitalRead(RANKING_BUTTON_PIN)){
        orderByLowAlgorithm();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("BEST : ");
        lcd.print(g_ranking[0]);
      }
    }
  } else if (g_btimerControllerStarted) {
    g_timerStart = millis();
    unsigned long syncTimerStart, syncTimerEnd, timerStart;
    syncTimerStart = millis();
    timerStart = millis();
    float timerValue;
    while (g_btimerControllerStarted) {
      syncTimerEnd = millis();
      if (syncTimerEnd - syncTimerStart >= 500) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Timer : ");
        timerValue = (float)(syncTimerEnd - timerStart) / 1000.0;
        lcd.print(timerValue);
        syncTimerStart = millis();
      } else {
        int servoAngleX = map(getJoystickDataLocationX(), 0, 1023, set_maxAngle, 0);
        int servoAngleY = map(getJoystickDataLocationY(), 0, 1023, 0, set_maxAngle);

        moveLocationX(servoAngleX);
        moveLocationY(servoAngleY);
      }
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("End Timer : ");
    lcd.print(timerValue);
    g_ranking[g_arrayNum] = timerValue;
    g_arrayNum += 1;
    g_btimerControllerStarted = false;
    delay(5000);
  }
}


void initializing(){
  bool error = false;
  g_Xservo.write(180);
  delay(1000);
  if (absF(getLocationXServoAngle() - 180) > 10){
    error = true;
  }
  delay(500);
  g_Yservo.write(180);
  delay(1000);
  if (absF(getLocationYServoAngle() - 180) > 10){
    error = true;
  }
  moveLocationX(90);
  moveLocationY(90);
  Serial.println(getLocationXServoAngle());
  Serial.println(getLocationYServoAngle());
  delay(1000);

  if (error){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("initializing");
    lcd.setCursor(0, 1);
    lcd.print("Failed : Motor");
    while(true){
    }
  }
}



int getJoystickDataLocationX() {
  return analogRead(JOYSTICK_X_PIN_NUM);
}

int getJoystickDataLocationY() {
  return analogRead(JOYSTICK_Y_PIN_NUM);
}

void moveLocationX(int angle) {
  g_Xservo.write(angle);
  return;
}

void moveLocationY(int angle) {
  g_Yservo.write(angle);
  return;
}

int getLocationXServoAngle() {
  return g_Xservo.read();
}

int getLocationYServoAngle() {
  return g_Yservo.read();
}

bool is_pressedSwitch() {
  int data = digitalRead(START_INTERRUPT_PIN);
  if (data == 0) {
    return true;
  } else {
    return false;
  }
}

void Debug() {
  Serial.print("Joystick X = ");
  Serial.print(getJoystickDataLocationX());
  Serial.print(", JoystickY = ");
  Serial.print(getJoystickDataLocationY());
  Serial.print(", SW = ");
  Serial.print(is_pressedSwitch());
  if (absF(getLocationXServoAngle() - map(getJoystickDataLocationX(), 0, 1023, set_maxAngle, 0)) > 30 ||
      absF(getLocationYServoAngle() - map(getJoystickDataLocationY(), 0, 1023, 0, set_maxAngle)) > 30) {
    
    Serial.print(", Servo Angle X = ");
    Serial.print(getLocationXServoAngle());
    Serial.print(", Servo Angle Y = ");
    Serial.print(getLocationYServoAngle());
    Serial.println(" Hmm… Please check wired your servo motor with Arduino");
  } else {
    Serial.print(", Servo Angle X = ");
    Serial.print(getLocationXServoAngle());
    Serial.print(", Servo Angle Y = ");
    Serial.println(getLocationYServoAngle());
  }
}

int absF(int inputValue) {
  if (inputValue >= 0) {
    return inputValue;
  } else if (inputValue < 0) {
    return -inputValue;
  }
}

void orderByLowAlgorithm(){
  for (int i = 0; i < g_arrayNum; i++){
    for (int j = 0; j < g_arrayNum; j++){
      if (g_ranking[i] < g_ranking[j]){
        float tmp = g_ranking[i];
        g_ranking[i] = g_ranking[j];
        g_ranking[j] = tmp;
      }
    }
  }
}
