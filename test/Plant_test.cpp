#include <Arduino.h>
#include <esp32_hardware.h>
#include <esp32_Encoder.h>
#include <ESP32Servo.h>
#include <motor.h>

void rotateAngle(float angle, bool direction);
bool moveLeadToPositionMM(float target_mm, int speed, bool reset_encoder);
void plant_cabbage();
bool isLimitHit();

Controller motor(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR_INV, MOTOR_BRAKE, MOTOR_PWM, MOTOR_IN_A, MOTOR_IN_B);
esp32_Encoder encoder(MOTOR_ENCODER_PIN_A, MOTOR_ENCODER_PIN_B,
					  ENCODER_PULSES_PER_REVOLUTION, MOTOR_ENCODER_INV, MOTOR_ENCODER_RATIO, WHEEL_DIAMETER);
Servo servo;

// ถ้าเป็นมอเตอร์ 1.8°/step = 200 step/รอบ
const int stepsPerRevolution = 6400;
float max_val = 1023.0;
float min_val = -max_val;


void setup() {
  Serial.begin(115200);

  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);  // ปกติ HIGH, ชน = LOW

  servo.attach(SERVO_PIN, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  pinMode(ENA, OUTPUT);

  digitalWrite(ENA, LOW);   // บาง driver LOW = enable

//   rotateAngle(30, true);    // หมุน 30° ตามเข็ม
//   delay(1000);
//   plant_cabbage();
}

void loop() {
  plant_cabbage();
  delay(5000);
}

void rotateAngle(float angle, bool direction) {

  int steps = round((stepsPerRevolution * angle) / 360.0);

  digitalWrite(DIR, direction);
  delay(5);   // ให้ DIR เซ็ตตัวก่อน

  for (int i = 0; i < steps; i++) {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(1500);
    digitalWrite(PUL, LOW);
    delayMicroseconds(1500);
  }
}

bool moveLeadToPositionMM(float target_mm, int speed = 800, bool reset_encoder = false) {
    // --- Conversion factor ---
    const float counts_per_mm =
        (ENCODER_COUNTS_PER_REV * MOTOR_ENCODER_RATIO * LEAD_SCREW_GEAR_RATIO) / LEAD_SCREW_PITCH_MM_PER_REV;

    // --- Clamp target to safe range (optional) ---
    // const float MIN_MM = 0.0;
    // const float MAX_MM = 300.0;
    // target_mm = constrain(target_mm, MIN_MM, MAX_MM);

    // --- Convert target mm → encoder counts ---
    long target_counts = target_mm * counts_per_mm;

    // --- Read current position ---
    long current_counts = encoder.read();

    // --- Error ---
    long error_counts = target_counts - current_counts;
    float error_mm = error_counts / counts_per_mm;
    Serial.println("Target mm: " + String(target_mm) + ", Current mm: " + String(current_counts / counts_per_mm) + ", Error mm: " + String(error_mm));

    // --- Stop condition ---
    const float error_tolerance_mm = 0.10;   // ตั้ง tolerance

    if (error_mm > 0 && isLimitHit() && reset_encoder) {
    motor.spin(0);
    encoder.reset();
    return true;
    }

    if (abs(error_mm) <= error_tolerance_mm) {
        motor.spin(0);
        return true;
    }

    if (error_mm > 0) {
        // move forward
        motor.spin(speed);
    } else {
        // move backward
        motor.spin(-speed);
    }
    delay(10);
    return false;
}

void plant_cabbage(){
    while(!moveLeadToPositionMM(1000.0f, 900, true));
    rotateAngle(30, true);    // หมุน 30° ตามเข็ม
    delay(1000);
    // set starting count value after attaching
    servo.writeMicroseconds(1235);
	encoder.reset();
    // while(!moveLeadToPositionMM(-20.0f, 700));
    while(!moveLeadToPositionMM(-195.0f));
	encoder.reset();
    // while(!moveLeadToPositionMM(5.0f));
    while(!moveLeadToPositionMM(12.5f));
    while(!moveLeadToPositionMM(50.0f)){
        servo.writeMicroseconds(900);
    }
    delay(750);
    servo.writeMicroseconds(1280);
    delay(1000);
    while(!moveLeadToPositionMM(0.0f, 1023));
    

    for(int i=0; i<2; i++){
        while(!moveLeadToPositionMM(5.0f));
        while(!moveLeadToPositionMM(25.0f, 1023)){
            servo.writeMicroseconds(900);
        }
        servo.writeMicroseconds(1280);
        delay(500);
        while(!moveLeadToPositionMM(0.0f, 1023));
        // servo.writeMicroseconds(1235);
        delay(500);
    }


    while(!moveLeadToPositionMM(5.0f));
    while(!moveLeadToPositionMM(25.0f, 650)){
        servo.writeMicroseconds(900);
    }
    while(!moveLeadToPositionMM(0.0f));
    servo.writeMicroseconds(1050);
    delay(800);
    servo.writeMicroseconds(750);
    delay(500);
    servo.writeMicroseconds(1100);
    delay(800);
    servo.writeMicroseconds(750);
    delay(500);
    // while(!moveLeadToPositionMM(5.0f));
    while(!moveLeadToPositionMM(60.0f, 1023));
    // while(!moveLeadToPositionMM(170.0f));
    servo.writeMicroseconds(1225);
    while(!moveLeadToPositionMM(195.0f, 900, true));

	// Serial.println("Encoder Start = " + String((int32_t)encoder.read()));
}

bool isLimitHit() {
    return digitalRead(LIMIT_SWITCH_PIN) == LOW;
}

