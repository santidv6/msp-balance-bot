#include <Wire.h>
//PIN NAMES
#define MOTORS_FW_PIN   7
#define MOTORS_BW_PIN   8
#define MOTORS_PWM_PIN  9
//MPU ADDRESS
#define MPU_ADDR        0x68  // Set address for MPU-6050: AD0 pin=0 -> 0x68, AD0 pin=1 -> 0x69
//MPU REGISTERS ADDRESSES
#define CONFIG          0x1A
#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40
#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42
#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48
#define PWR_MGMT_1      0x6B
//REGISTER VALUES
#define DLPF_1          0x01    // 184/188 Hz  ACCELEROMETER AND GYROSCOPE
#define DLPF_2          0x02    // 94/98 Hz    DIGITAL
#define DLPF_3          0x03    // 44/42 Hz    LOW
#define DLPF_4          0x04    // 21/20 Hz    PASS
#define DLPF_5          0x05    // 10/10 Hz    FILTER
#define DLPF_6          0x06    // 5/5 Hz      CONFIGURATION
//SYSTEM PARAMETERS AND CONVERSION FACTORS
#define RAD_DEG         (180/PI)
#define MDZ_OFFSET      20      // MOTORS (MOVING) DEAD ZONE OFFSET
#define MAX_PWM         255
#define GYRO_SCALE      131.0
#define ACCEL_SCALE     16384.0
#define KP              (4.0*2.55)//(3.8*2.55)//(3.6*2.55)
#define KI              (7.0*2.55)//(7.0*2.55)//(7.0*2.55)  // Ti = Kp/Ki
#define KD              (0.065*2.55)//(0.06*2.55)//(0.05*2.55) // Td = Kd/Kp
#define CF_ALPHA        0.98

//GLOBAL VARIABLES
uint8_t data;
int16_t accel_x, accel_y, accel_z;
int16_t temp;
int16_t gyro_x, gyro_y, gyro_z;

float accel_scaled_x, accel_scaled_y, accel_scaled_z;
float gyro_scaled_x, gyro_scaled_y, gyro_scaled_z;
float rotation_x, rotation_y;
float accel_offset_x = 550;
float gyro_offset_x, gyro_offset_y;
float gyro_x_delta, gyro_y_delta;
float gyro_total_x, gyro_total_y;
float CF_x, CF_y;
float prev_CF_x, prev_CF_y;

float ref = 2;
float error, prev_error = 0;
float error_inc, error_sum = 0;
float pid_output, output;

float tm = 0.008;
uint32_t init_time, current_time, prev_time = 0;
uint8_t tm_count = 0;
float tm_sum = 0;
boolean dim_sum_flag = false;

//FUNCTION PROTOTYPES
void set_normal_mode();
void set_DLPF(uint8_t value);
void read_all();
void print_all(boolean plotter, boolean raw);
void print_angle_error_pid(boolean plotter);
void print_cfdata();

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  pinMode(MOTORS_FW_PIN, OUTPUT);
  pinMode(MOTORS_BW_PIN, OUTPUT);
  pinMode(MOTORS_PWM_PIN, OUTPUT);
  digitalWrite(MOTORS_FW_PIN, LOW);
  digitalWrite(MOTORS_BW_PIN, LOW);

#if MAX_PWM == 4095
  analogResolution(4096);
#endif
  analogFrequency(1000);

  set_normal_mode();
  set_DLPF(DLPF_3);
  delay(100);

  //FIRST EXECUTION
  init_time = millis();
  for (int i = 0; i < 10; i++) {  // AVERAGE THE STARTING POINT
    prev_time = millis();
    read_all();
    //print_all(false, false);
    //rotation_x = atan2(accel_scaled_y, sqrt(pow(accel_scaled_x, 2) + pow(accel_scaled_z, 2))) * RAD_DEG;
    rotation_y += atan2(-1*accel_scaled_x, sqrt(pow(accel_scaled_y, 2) + pow(accel_scaled_z, 2))) * RAD_DEG;
    //gyro_offset_x = gyro_scaled_x;
    gyro_offset_y += gyro_scaled_y;
  }
  rotation_y = rotation_y / 10.0;
  gyro_offset_y = gyro_offset_y / 10.0;

  //CF_x = rotation_x;
  //gyro_total_x = CF_x;
  CF_y = rotation_y;
  gyro_total_y = CF_y;
}

void loop()
{
  current_time = millis();
  tm = (current_time - prev_time) / 1000.0;
  prev_time = current_time;
//  if (tm_count < 50) {
//    tm_sum += tm;
//    tm_count++;
//  } 
//  else {
//    Serial.print("tm_average: "); 
//    Serial.println(tm_sum/50.0,4); // We can get 5.6 ms of control loop wo/ prints
//  }
  //Serial.print("tm: "); 
  //Serial.println(tm,3);
  read_all();
  //print_all(false, false);

  //gyro_scaled_x -= gyro_offset_x;
  gyro_scaled_y -= gyro_offset_y;
  //gyro_x_delta = (gyro_scaled_x * tm);
  gyro_y_delta = (gyro_scaled_y * tm);
  //gyro_total_x = CF_x + gyro_x_delta;
  gyro_total_y += gyro_y_delta;
  //rotation_x = atan2(accel_scaled_y, sqrt(pow(accel_scaled_x, 2) + pow(accel_scaled_z, 2))) * RAD_DEG;
  rotation_y = atan2(-1*accel_scaled_x, sqrt(pow(accel_scaled_y, 2) + pow(accel_scaled_z, 2))) * RAD_DEG;
  //CF_x = CF_ALPHA * gyro_total_x + (1 - CF_ALPHA)s * rotation_x;
  //error = ref - CF_x;
  CF_y = CF_ALPHA * (CF_y + gyro_y_delta) + (1 - CF_ALPHA) * rotation_y;
  error = ref - CF_y;

  print_cfdata();

  error_inc = error - prev_error;                      //Derivative term calculation
  error_sum = error_sum + (error + prev_error) / 2.0;  //Integral term calculation (Tustin)
  if (error * prev_error < 0 && error_sum > 1200) {    //Pseudo windup reduction mechanism
    error_sum *= 0.87;
  }
  prev_error = error;
  //Serial.println(error_sum);

  if ((CF_y < -35) || (CF_y > 35)) {    //stop motors in case of falling
    digitalWrite(MOTORS_FW_PIN, LOW);
    digitalWrite(MOTORS_BW_PIN, LOW);
    analogWrite(MOTORS_PWM_PIN, 0);
    error_sum = 0;
  } 
  else {
    pid_output = KP * error + KD * (error_inc / tm) + KI * (error_sum * tm);
    if (pid_output > 5 && pid_output < MDZ_OFFSET) {
      pid_output = MDZ_OFFSET;
    } 
    else if (pid_output < -5 && pid_output > -MDZ_OFFSET) {
      pid_output = -MDZ_OFFSET;
    }
    //SATURATION (PRE-PRINT)
    if (pid_output > 255) {
      pid_output = 255;
    } 
    else if (pid_output < -255) {
      pid_output = -255;
    }
    //print_angle_error_pid(true);

    output = (int)(abs(pid_output));

    if (pid_output < 0) {
      digitalWrite(MOTORS_FW_PIN, LOW);
      digitalWrite(MOTORS_BW_PIN, HIGH);
    } 
    else if (pid_output > 0) {
      digitalWrite(MOTORS_FW_PIN, HIGH);
      digitalWrite(MOTORS_BW_PIN, LOW);
    }
    analogWrite(MOTORS_PWM_PIN, output);
  }
}

void set_normal_mode() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(byte(PWR_MGMT_1));
  Wire.write(byte(0));
  Wire.endTransmission();
  delay(10);
}

void set_DLPF(uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(byte(CONFIG));
  Wire.write(byte(value));
  Wire.endTransmission();
  delay(10);
}

void read_all() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(byte(ACCEL_XOUT_H));
  Wire.endTransmission();

  Wire.requestFrom(MPU_ADDR, 14);
  
  while(Wire.available() < 14);
  
  data = Wire.read();
  accel_x = data << 8;
  accel_x |= Wire.read(); 
  accel_x -= accel_offset_x;
  data = Wire.read();
  accel_y = data << 8;
  accel_y |= Wire.read();
  data = Wire.read();
  accel_z = data << 8;
  accel_z |= Wire.read();
  data = Wire.read();
  temp = data << 8;
  temp |= Wire.read();
  //temp = temp/340 + 36.53;
  data = Wire.read();
  gyro_x = data << 8;
  gyro_x |= Wire.read();
  data = Wire.read();
  gyro_y = data << 8;
  gyro_y |= Wire.read();
  data = Wire.read();
  gyro_z = data << 8;
  gyro_z |= Wire.read();

  accel_scaled_x = accel_x / ACCEL_SCALE;
  accel_scaled_y = accel_y / ACCEL_SCALE;
  accel_scaled_z = accel_z / ACCEL_SCALE;
  gyro_scaled_x = gyro_x / GYRO_SCALE;
  gyro_scaled_y = gyro_y / GYRO_SCALE;
  gyro_scaled_z = gyro_z / GYRO_SCALE;
}

void print_all(boolean plotter, boolean raw) {
  if (raw) {
    if (plotter) {
      Serial.println("accel_x  accel_y  accel_z  gyro_x   gyro_y   gyro_z");
      Serial.print(accel_x);  
      Serial.print("    ");
      Serial.print(accel_y);  
      Serial.print("    ");
      Serial.print(accel_z);  
      Serial.print("    ");
      //Serial.print(temp,4);     Serial.print("    ");
      Serial.print(gyro_x);   
      Serial.print("    ");
      Serial.print(gyro_y);   
      Serial.print("    ");
      Serial.println(gyro_z);
    } 
    else {
      Serial.print("accel_x: "); 
      Serial.print(accel_x);
      Serial.print(" accel_y: "); 
      Serial.print(accel_y);
      Serial.print(" accel_z: "); 
      Serial.print(accel_z);
      //    Serial.print(" temp: "); Serial.print(temp,4);
      Serial.print(" gyro_x: "); 
      Serial.print(gyro_x);
      Serial.print(" gyro_y: "); 
      Serial.print(gyro_y);
      Serial.print(" gyro_z: "); 
      Serial.println(gyro_z);
    }
  } 
  else {
    if (plotter) {
      Serial.println("accel_scaled_x  accel_scaled_y  accel_scaled_z  gyro_scaled_x   gyro_scaled_y   gyro_scaled_z");
      Serial.print(accel_scaled_x,4);  
      Serial.print(" ");
      Serial.print(accel_scaled_y,4);  
      Serial.print(" ");
      Serial.print(accel_scaled_z,4);  
      Serial.print(" ");
      //Serial.print(temp,4);     Serial.print("    ");
      Serial.print(gyro_scaled_x,4);   
      Serial.print(" ");
      Serial.print(gyro_scaled_y,4);   
      Serial.print(" ");
      Serial.println(gyro_scaled_z,4);
    } 
    else {
      Serial.print("accel_scaled_x: "); 
      Serial.print(accel_scaled_x,4);
      Serial.print(" accel_scaled_y: "); 
      Serial.print(accel_scaled_y,4);
      Serial.print(" accel_scaled_z: "); 
      Serial.print(accel_scaled_z,4);
      //    Serial.print(" temp: "); Serial.print(temp,4);
      Serial.print(" gyro_scaled_x: "); 
      Serial.print(gyro_scaled_x,4);
      Serial.print(" gyro_scaled_y: "); 
      Serial.print(gyro_scaled_y,4);
      Serial.print(" gyro_scaled_z: "); 
      Serial.println(gyro_scaled_z,4);
    }
  }
}

void print_angle_error_pid(boolean plotter) {
  if (plotter) {
    //Serial.println("CF_y error pid_output");
    Serial.print((current_time-init_time)/1000.0,3); //only for plotting in Matlab, comment out if using Arduino's Plotter
    Serial.print(" ");                 //only for plotting in Matlab, comment out if using Arduino's Plotter
    Serial.print(CF_y,3); 
    Serial.print(" ");
    Serial.print(error,3); 
    Serial.print(" ");
    Serial.println(pid_output,3);
  } 
  else {
    Serial.print("CF_y: "); 
    Serial.print(CF_y,3);
    Serial.print(" error: "); 
    Serial.print(error,3);
    Serial.print(" pid_output: "); 
    Serial.println(pid_output,3);
  }
}

void print_cfdata() {
  //Serial.println("rotation_y  gyro_total_y CF_y");
  //Serial.print((current_time-init_time)/1000.0,3);
  Serial.print(rotation_y,4);  
  Serial.print(" ");
  Serial.print(gyro_total_y,4);  
  Serial.print(" ");
  Serial.print(CF_y,4);  
  Serial.println(" ");
}








