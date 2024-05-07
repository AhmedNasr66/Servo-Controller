#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>

// Define PCA9685 parameters
#define PCA9685_ADDR 0x40  // Default I2C address of PCA9685
#define PCA9685_FREQ 50    // PWM frequency for servos (typically 50Hz)

// Define Bluetooth serial instance
BluetoothSerial SerialBT;

// Create PCA9685 object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(57600);

  // Initialize Bluetooth serial
  SerialBT.begin("ESP32_Servo_Control");

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(PCA9685_FREQ);

  // Print startup message
  Serial.println("ESP32 Bluetooth PCA9685 Servo Control");
}

void loop() {
  // Check if any data is received via Bluetooth
  if (SerialBT.available() >= 2) {
    unsigned int servo_pos = SerialBT.read();
    unsigned int servo_pos1 = SerialBT.read();

    unsigned int servo_real_pos = (servo_pos1 * 256) + servo_pos;

    Serial.print("Servo  position: ");
    Serial.println(servo_real_pos);

    if (servo_real_pos >= 1000 && servo_real_pos <= 1180) {
      int servo_1 = map(servo_real_pos - 1000, 0, 180, 500, 2500);
      int pwm_value_1 = map(servo_1, 0, 20000, 0, 4095);
      pwm.setPWM(0, 0, pwm_value_1);  // Set servo 1 position on channel 0
      Serial.println("Servo 1 ON");
      Serial.println("Servo 1 angle is");
      Serial.println(servo_real_pos - 1000);
    }

    if (servo_real_pos >= 2000 && servo_real_pos <= 2180) {
      int servo_2 = map(servo_real_pos - 2000, 0, 180, 500, 2500);
      int pwm_value = map(servo_2, 0, 20000, 0, 4095);
      pwm.setPWM(1, 0, pwm_value);  // Set servo 2 position on channel 1
      Serial.println("Servo 2 ON");
      Serial.println("Servo 2 angle is");
      Serial.println(servo_real_pos - 2000);
    }

    if (servo_real_pos >= 3000 && servo_real_pos <= 3180) {
      int servo_3 = map(servo_real_pos - 3000, 0, 180, 500, 2500);
      int pwm_value_2 = map(servo_3, 0, 20000, 0, 4095);
      pwm.setPWM(2, 0, pwm_value_2);  // Set servo 3 position on channel 2
      Serial.println("Servo 3 ON");
      Serial.println("Servo 3 angle is");
      Serial.println(servo_real_pos - 3000);
    }
    if (servo_real_pos >= 4000 && servo_real_pos <= 4180) {
      int servo_4 = map(servo_real_pos - 4000, 0, 180, 500, 2500);
      int pwm_value_4 = map(servo_4, 0, 20000, 0, 4095);
      pwm.setPWM(3, 0, pwm_value_4);  // Set servo 4 position on channel 3
      Serial.println("Servo 3 ON");
      Serial.println("Servo 3 angle is");
      Serial.println(servo_real_pos - 4000);
    }

    if (servo_real_pos >= 6000 && servo_real_pos <= 6180) {
      int Elevator = map(servo_real_pos - 6000, 0, 180, 500, 2500);
      int Elevatorpwm = map(Elevator, 0, 20000, 0, 4095);
      pwm.setPWM(1, 0, Elevatorpwm);
      pwm.setPWM(0, 0, Elevatorpwm);  // Set servo 4 position on channel 3
      Serial.println("Servo 3 ON");
      Serial.println("Servo 3 angle is");
      Serial.println(servo_real_pos - 6000);
    }

    if (servo_real_pos >= 7000 && servo_real_pos <= 7180) {
      int Rudder = map(servo_real_pos - 7000, 0, 180, 500, 2500);
      int Rudderpwm = map(Rudder, 0, 20000, 0, 4095);
      pwm.setPWM(2, 0, Rudderpwm);  // Set servo 4 position on channel 3
      pwm.setPWM(3, 0, Rudderpwm);
      Serial.println("Rudder ON");
      Serial.println("Rudder angle is");
      Serial.println(servo_real_pos - 7000);
    }

    if (servo_real_pos >= 8000 && servo_real_pos <= 8025) {
      int Allservos = map(servo_real_pos - 8000, 0, 180, 500, 2500);
      int Allservospwm = map(Allservos, 0, 20000, 0, 4095);
      pwm.setPWM(0, 0, Allservospwm);  // Set servo 1 position on channel 0
      pwm.setPWM(1, 0, Allservospwm);  // Set servo 2 position on channel 1
      pwm.setPWM(2, 0, Allservospwm);  // Set servo 3 position on channel 2
      pwm.setPWM(3, 0, Allservospwm);  // Set servo 4 position on channel 3
      Serial.println("All servos ON");
      Serial.println("Servos angle is");
      Serial.println(servo_real_pos - 7000);
    }
    // This section for Testing Servos motion
    if (servo_real_pos == 5000) {
      int servo_test_1 = map(180, 0, 180, 500, 2500);
      int pwm_value_test1 = map(servo_test_1, 0, 20000, 0, 4095);
      pwm.setPWM(0, 0, pwm_value_test1);  // Set servo 1 position on channel 0



      int servo_test_2 = map(180, 0, 180, 500, 2500);
      int pwm_value_test2 = map(servo_test_2, 0, 20000, 0, 4095);
      pwm.setPWM(1, 0, pwm_value_test2);  // Set servo 2 position on channel 1



      int servo_test_3 = map(180, 0, 180, 500, 2500);
      int pwm_value_test3 = map(servo_test_3, 0, 20000, 0, 4095);
      pwm.setPWM(2, 0, pwm_value_test3);  // Set servo 3 position on channel 2



      int servo_test_4 = map(180, 0, 180, 500, 2500);
      int pwm_value_test4 = map(servo_test_4, 0, 20000, 0, 4095);
      pwm.setPWM(3, 0, pwm_value_test4);  // Set servo 4 position on channel 3


      delay(2000);

      servo_test_1 = map(0, 0, 180, 500, 2500);
      pwm_value_test1 = map(servo_test_1, 0, 20000, 0, 4095);
      pwm.setPWM(0, 0, pwm_value_test1);  // Set servo 1 position on channel 0



      servo_test_2 = map(0, 0, 180, 500, 2500);
      pwm_value_test2 = map(servo_test_2, 0, 20000, 0, 4095);
      pwm.setPWM(1, 0, pwm_value_test2);  // Set servo 2 position on channel 1



      servo_test_3 = map(0, 0, 180, 500, 2500);
      pwm_value_test3 = map(servo_test_3, 0, 20000, 0, 4095);
      pwm.setPWM(2, 0, pwm_value_test3); // Set servo 3 position on channel 2



      servo_test_4 = map(0, 0, 180, 500, 2500);
      pwm_value_test4 = map(servo_test_4, 0, 20000, 0, 4095);
      pwm.setPWM(3, 0, pwm_value_test4); // Set servo 4 position on channel 3
    }
  }


  delay(5);
}
