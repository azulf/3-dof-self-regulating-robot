#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include <LiquidCrystal_I2C.h>

//#define PAKAI_LCD
#define PAKAI_HCSR
#define PAKAI_MPU
#define C_FILTER
#define K_INVERSE
#define PID
#define OUTPUT_DATA
// #define TIDAK_PAKAI_SERVO


// LCD
LiquidCrystal_I2C lcd (0x27, 16, 2);

// SERVO
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
#define MAX_ANGLE_SERVO  180
#define MIN_ANGLE_SERVO  0

//  Global Declaration Variable for MPU6050
long accX, accY, accZ, accTotal_vector;
int gyroX, gyroY, gyroZ;
int temperature;
long gyroX_cal, gyroY_cal, gyroZ_cal;
long loopTimer;
float anglePitch, angleRoll;
int anglePitch_buffer, angleRoll_buffer;
boolean setGyro_angles;
float angleRoll_acc, anglePitch_acc;
float anglePitch_output, angleRoll_output;
int i = 1;
int N = 1000;
float offset_anglePitch_acc, offset_angleRoll_acc, sum_anglePitch_acc,sum_angleRoll_acc;
float hitung = 0;
float Ts = 0.004; //0.01 -> (100 Hz), 0.004 (250Hz)
int doneCal = 0;
int filterConfig = 1;

// nilai servo baru
int ServoYaw;
int ServoPitch;

// =========================================================================================== //
// Global Declaration for Kinematics 3 dof Inverse kinematics 
int countGerak = 0;
const float L1 = 15.3;  // dalam cm
const float L2 = 15.3;  // dalam cm
const float L3 = 7;     // dalam cm
const float phi = 3.145;
float x,y,z; // -> nilai x,z tampak samping. Nilai x,y tampak atas
float theta1,theta2,theta3,theta4;
float alpha1,alpha2,alpha3;
float beta1,beta2,beta3;
float a,b,c;
float q_deg1,q_deg2,q_deg3;
float movs1,movs2,movs3;
float angleOutput1, angleOutput2, angleOutput3;

// ======================================================================================= //

// Global Declaration for HC-SR Ultrasonic Module
const int trigPin = 12;
const int echoPin = 13;
long durasi;
float jarak;

// Yaw, Pitch Values
float Yaw;
float Pitch;
int j = 0;
float correct;

// Variable PID
float spJarak = 13; // --> nilai setpoint ketinggian yang diinginkan

float Kp = 1.18;
float Ki = 0.67;
float Kd = 0.15;

float error,errorx,sumerr;
double PIDxJarak = 0;
double errorJarak;
double lastErrorJarak;
double arahJarak; // --> atas & turun
double StartTime;
double PIDOutput;

int previousTimePrint = 0;

// long duration, distance --> sudah ada variablenya pada HC-SR


// ======================================================================================= //
// Setup    
void setup() {
  // inisialisasi I2C
  Wire.begin();

  // inisialisasi lcd screen
  lcd.init();
  lcd.backlight();

  // inisialisasi MPU
  Serial.begin(57600);
  pinMode(13, OUTPUT);

// AKTIFKAN JIKA MENGGUNAKAN KALIBRASI
#ifdef PAKAI_MPU
  setupMPURegisters();
  rawMPU6050();
#endif
  // inisialisasi pin servo
  servo1.attach(3);
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(8);
  servo5.attach(9);

  // #ifdef TIDAK_PAKAI_SERVO
  // funcNotMovingServo();     // DINYALAKAN KETIKA SERVO TIDAK DIGUNAKAN
  // #endif

  // inisialisasi hc-sr04
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // gerakan awal robot
  robotGerakanAwal(); 
  previousTimePrint = millis();
}

  // Loop
void loop() {
  StartTime = millis();
// Kalibrasi nilai 0 pada MPU dan perhitungan complementary filter
#ifdef PAKAI_MPU
  while (doneCal == 0)  // fungsi nesting untuk kalibrasi
  {
    MPUCalibrate();
      if (filterConfig == 1)
      {
        FilterCompMPU();
      }
  }
#endif  
  MPUJalan();
  delay(50);
  Ultrasonic();
  selfRegH();
  YawPitch();


  #ifdef PAKAI_LCD
    lcdOutput();
  #endif

  #ifdef OUTPUT_DATA
    outputFunc();
  #endif
  // funcAuto();
  funcMovingServo();
}

void robotGerakanAwal()
{
  servo1.write(70);
  servo2.write(50);
  servo3.write(80);
  servo4.write(75);
  servo5.write(150);
  kInverseCalculate(0, 1);
  delay(2000);
}

void funcAuto()
{
  MPUJalan();
  delay(50);
  Ultrasonic();
  selfRegH();
  YawPitch();


  #ifdef PAKAI_LCD
    lcdOutput();
  #endif

  #ifdef OUTPUT_DATA
    outputFunc();
  #endif
}

void funcNotMovingServo()
{
  servo1.detach();
  servo2.detach();
  servo3.detach();
  servo4.detach();
  servo5.detach();
}

void funcMovingServo()
{
  servo1.write(angleOutput1); 
  servo2.write(angleOutput2);
  servo3.write(angleOutput3); 
  servo4.write(ServoYaw);
  servo5.write(ServoPitch);
}



// Fungsi Input Ultrasonik HC-SR04
void Ultrasonic()
{
  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);
  // Baca nilai echo, sinyal kembali pada microsekon
  durasi = pulseIn(echoPin,HIGH);
  // Kalkulasi nilai jarak
  jarak = durasi * 0.034 / 2;
  if (jarak >= 25)
  {
    jarak = 25;
  }
  if (jarak <= 0)
  {
    jarak = 0;
  }
  // Serial.print("Jarak :");
  // Serial.println(jarak);

  // Nilai masukan PID
  calculatePID();
}

// fungsi return nilai perubahan kinematik inverse dari input HC-SR04
void selfRegH()
{
  if (jarak >= 25)
  {
    jarak = 25;
  }
  if (jarak <= 0)
  {
    jarak = 0;
  }
  if (spJarak - jarak < 0)
  {
    arahJarak = -1;
  }

  else if (spJarak - jarak > 0)
  {
    arahJarak = 1;
  }

  float deltaH = (spJarak - jarak);
  if (arahJarak < 0) 
  {
    // deltaH = (deltaH * arahJarak) + spJarak;
    deltaH = spJarak + deltaH;
  }
  else if (arahJarak > 0)
  {
    deltaH = spJarak - (deltaH * -1 );
  }

  if (deltaH >= 1) // bergerak ketika nilai ketinggian H atau Z diatas 3.5 cm
  { 
    kInverseCalculate(deltaH, arahJarak);
  }
  else 
  {
    kInverseCalculate(spJarak, arahJarak);
  }
}


#ifdef PID
void calculatePID()
{
  // PID untuk posisi ketinggian
  errorJarak = abs((double)jarak - (double)spJarak);
  double CurrentTime = millis();
  double elapsedTime = CurrentTime - StartTime;

  lastErrorJarak = errorJarak;
  if (lastErrorJarak != 0)
  {
    PIDxJarak = (Kp * errorJarak) + (Ki *(errorJarak/elapsedTime)) + (Kd * (errorJarak - lastErrorJarak)/elapsedTime);  
  }

  else {
    PIDxJarak = 0;
  }
}
#endif

void kInverseCalculate(float deltaZ, double signDist)
{
  x = 24;
  // Input X and Y Coordinates and theta assumption
  z = deltaZ;
  Serial.println(z);

  b = x - L3; // mencari panjang b sebagai jarak langsung dari servo base ke servo 3
  c = sqrt((b*b)+(z*z)); // dengan menganggap nilai sudut servo 3 90 derajat
  

  // Mencari nilai Theta2 atau nilai sudut servo base
  alpha1 = atan(z/b);
  alpha2 = acos(((L1*L1)+(c*c)-(L2*L2))/(2*L1*c));
  theta2 = alpha1 + alpha2;

  // Mencari nilai theta3 atau nilai sudut servo 2
  theta3 = acos(((L1*L1)+(L2*L2)-(c*c))/(2*L1*L2));

  // theta 4 dianggap 90 derajat sehingga membentuk 3 sudut
  // beta1,beta2,beta3
  beta1 = acos(((L2*L2)+(c*c)-(L1*L1))/(2*L2*c));
  beta2 = atan(b/z);
  // beta3 = 1.5708;
  beta3 = 90*phi/180;

  theta4 = beta1 + beta2 + beta3;
  // hasil dalam bentuk derajat tanpa diubah ke radian
  q_deg1 = theta2*180/phi;
  q_deg2 = theta3*180/phi;
  q_deg3 = theta4*180/phi;
  if(q_deg3 >= 180)
  {
    q_deg3 = abs(q_deg3 - 90);  // masih dicari problem tapi sudah bisa
  }
  q_deg2 = abs(q_deg2 - 90);  // dikurang 90 karena penempatan sudut 0 servo pada robot

  PIDOutput = PIDxJarak * signDist;


  angleOutput1 = q_deg1 + (PIDxJarak * signDist);
  angleOutput2 = q_deg2 + (PIDxJarak * signDist);
  angleOutput3 = q_deg3 + (PIDxJarak * signDist);

    // BATASAN MINIMUM SERVO BERGERAK
  if (angleOutput1 <= 0 )
  {
    angleOutput1  = MIN_ANGLE_SERVO;
  }
  if (angleOutput2 <= 0 )
  {
    angleOutput2  = MIN_ANGLE_SERVO;
  }
  if (angleOutput3 <= 0 )
  {
    angleOutput3 = MIN_ANGLE_SERVO;
  }

  // BATASAN MAKSIMAL SERVO BERGERAK
  if (angleOutput1 >= 180 )
  {
    angleOutput1  = MAX_ANGLE_SERVO;
  }
  if (angleOutput2 >= 180 )
  {
    angleOutput2  = MAX_ANGLE_SERVO;
  }
  if (angleOutput3 >= 180 )
  {
    angleOutput3 = MAX_ANGLE_SERVO;
  }

}

#ifdef OUTPUT_DATA
void outputFunc()
{
  // int timenowPrint = millis();
  // int intervalPrint = 500;

  // if (timenowPrint - intervalPrint >= previousTimePrint )
  // {
  // OUTPUT NILAI PID ULTRASONIC
  Serial.print("Nilai Jarak cm = ");
  Serial.print(jarak);
  Serial.println("\t");
  Serial.print("Nilai PIDx = ");
  Serial.print(PIDxJarak);
  Serial.println("\t");

  // OUTPUT NILAI SERVO
  Serial.print("Servo Value 1 , 2 , 3 : ");
  Serial.print(angleOutput1);
  Serial.print(",\t");
  Serial.print(angleOutput2);
  Serial.print(",\t");
  Serial.print(angleOutput3);
  Serial.println("\t");

  // OUTPUT NILAI YAW PITCH SERVO 4 & 5
  Serial.print("Nilai Servo Yaw  = ");
  Serial.print(ServoYaw);
  Serial.print("\t");
  Serial.print("Nilai Servo Pitch = ");
  Serial.print(ServoPitch);
  Serial.println("");
  // previousTimePrint = timenowPrint;
  // }
}
#endif

void setupMPURegisters()
{
  // Reset nilai bit register MPU6050
  Wire.beginTransmission(0x68); // -> Gyro,Accel,Temp Reset
  Wire.write(0x6B);             // -> Device Reset from Sleep to cycle
  Wire.write(0x00);             // -> Resetting bit to sleep
  Wire.endTransmission();     

  // Config Accelerometer
  Wire.beginTransmission(0x68); // Gyro,Accel,Temp Reset
  Wire.write(0x1C);             // Config Address Accelerometer 
  Wire.write(0x10);             // Self_test Accelerometer
  Wire.endTransmission();       // End transmission

  // Config Gyro
  Wire.beginTransmission(0x68); // Gyro,Accel,Temp Reset
  Wire.write(0x1B);             // Config Address FS_SEL1:0
  Wire.write(0x08);             // Set bit to 1 500Hz
  Wire.endTransmission();
}

void ReadMPUData()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68, 14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  accX = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  accY = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  accZ = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyroX = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyroY = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyroZ = Wire.read()<<8|Wire.read();    
}

// Input nilai raw data dari MPU6050
void rawMPU6050()
{
  Serial.println("  MPU-6050 IMU  ");
  Serial.println("      V1.0      ");
  Serial.println("Tempatkan MPU Pada tempat datar!!!");
  delay(1500);                                                         //Delay 1.5 second to display the text
  Serial.println("Calibrating gyro");
    for (int cal_int = 0; cal_int < 2000 ; cal_int++)
    {
      if(cal_int % 125 == 0)Serial.print(".");                              //Print a dot on the LCD every 125 readings
    ReadMPUData();                                             //Read the raw acc and gyro data from the MPU-6050
    gyroX_cal += gyroX;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyroY_cal += gyroY;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyroZ_cal += gyroZ;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);  
    }
    gyroX_cal /= 2000;
    gyroY_cal /= 2000;
    gyroZ_cal /= 2000;

    Serial.println("\t");
    Serial.println(gyroX_cal);
    Serial.println(gyroY_cal);
    Serial.println(gyroZ_cal);
    Serial.println("DONE");
    delay(2000);
    loopTimer = micros();
}

void MPUCalibrate()
{
  ReadMPUData();                                                     //Read the raw acc and gyro data from the MPU-6050
  
  gyroX -= gyroX_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyroY -= gyroY_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyroZ -= gyroZ_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
    
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  anglePitch += gyroX * (1/(1/Ts)/65.5);                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angleRoll  += gyroY * (1/(1/Ts)/65.5);                                    //Calculate the traveled roll angle and add this to the angle_roll variable
    
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  anglePitch += angleRoll * sin(gyroZ * 1/(1/Ts)/65.5*3.142/180);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angleRoll  -= anglePitch * sin(gyroZ * 1/(1/Ts)/65.5*3.142/180);               //If the IMU has yawed transfer the pitch angle to the roll angel
    
  //Accelerometer angle calculations
  accTotal_vector = sqrt((accX*accX)+(accY*accY)+(accZ*accZ));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  anglePitch_acc = asin((float)accY/accTotal_vector)* 57.296;       //Calculate the pitch angle
  angleRoll_acc = asin((float)accX/accTotal_vector)* -57.296;       //Calculate the roll angle
    
  while(micros() - loopTimer < Ts*1000000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loopTimer = micros();                                               //Reset the loop timer
  
  Serial.print("pitch : ");
  Serial.print(anglePitch_acc);
  Serial.print("\t");
  Serial.print("roll : ");
  Serial.println(angleRoll_acc);
    
  sum_anglePitch_acc = sum_anglePitch_acc + anglePitch_acc;
  sum_angleRoll_acc = sum_angleRoll_acc + angleRoll_acc;

  if(i>=N)
  {
    offset_anglePitch_acc = sum_anglePitch_acc/N;
    offset_angleRoll_acc = sum_angleRoll_acc/N;

    Serial.print("Banyak data = "); Serial.println(i);
    Serial.print("Offset Angle Pitch ACC = ");
    Serial.println(offset_anglePitch_acc);
    Serial.print("Offset Angle Roll ACC = ");
    Serial.print(offset_angleRoll_acc);
    
    doneCal = 1;                 // batas fungsi sementara pada loop
  }
  i++;

  // Input nilai offset pada angle accelerometer

}

void FilterCompMPU()
{
   ReadMPUData();                                                     //Read the raw acc and gyro data from the MPU-6050
  
  gyroX -= gyroX_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyroY -= gyroY_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyroZ -= gyroZ_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
    
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  anglePitch += gyroX * (1/(1/Ts)/65.5);                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angleRoll  += gyroY * (1/(1/Ts)/65.5);                                    //Calculate the traveled roll angle and add this to the angle_roll variable
    
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  anglePitch += angleRoll * sin(gyroZ * 1/(1/Ts)/65.5*3.142/180);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angleRoll  -= anglePitch * sin(gyroZ * 1/(1/Ts)/65.5*3.142/180);               //If the IMU has yawed transfer the pitch angle to the roll angel
    
  //Accelerometer angle calculations
  accTotal_vector = sqrt((accX*accX)+(accY*accY)+(accZ*accZ));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  anglePitch_acc = asin((float)accY/accTotal_vector)* 57.296;       //Calculate the pitch angle
  angleRoll_acc = asin((float)accX/accTotal_vector)* -57.296;       //Calculate the roll angle

  anglePitch_acc -= offset_anglePitch_acc;
  angleRoll_acc -= offset_angleRoll_acc;

  if(setGyro_angles)
  {
    anglePitch = anglePitch * 0.91 + anglePitch_acc * 0.09;
    angleRoll = angleRoll * 0.87 + angleRoll_acc * 0.13;
  }
  else
  {
    anglePitch = anglePitch_acc;
    angleRoll = angleRoll_acc;
    setGyro_angles = true;
  }

  // nilai angle pitch dan roll yang difilter menggunakan complementary filter
  anglePitch_output = anglePitch_output * 0.9 + anglePitch * 0.1;
  angleRoll_output = angleRoll_output * 0.9 + angleRoll * 0.1;

  while (micros() - loopTimer < Ts*10000000);
  loopTimer = micros();

  filterConfig = 0;
}

void MPUJalan()
{
     ReadMPUData();                                                     //Read the raw acc and gyro data from the MPU-6050
  
  gyroX -= gyroX_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyroY -= gyroY_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyroZ -= gyroZ_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
    
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  anglePitch += gyroX * (1/(1/Ts)/65.5);                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angleRoll  += gyroY * (1/(1/Ts)/65.5);                                    //Calculate the traveled roll angle and add this to the angle_roll variable
    
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  anglePitch += angleRoll * sin(gyroZ * 1/(1/Ts)/65.5*3.142/180);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angleRoll  -= anglePitch * sin(gyroZ * 1/(1/Ts)/65.5*3.142/180);               //If the IMU has yawed transfer the pitch angle to the roll angel
    
  //Accelerometer angle calculations
  accTotal_vector = sqrt((accX*accX)+(accY*accY)+(accZ*accZ));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  anglePitch_acc = asin((float)accY/accTotal_vector)* 57.296;       //Calculate the pitch angle
  angleRoll_acc = asin((float)accX/accTotal_vector)* -57.296;       //Calculate the roll angle

  anglePitch_acc -= offset_anglePitch_acc;
  angleRoll_acc -= offset_angleRoll_acc;

  if(setGyro_angles)
  {
    anglePitch = anglePitch * 0.91 + anglePitch_acc * 0.09;
    angleRoll = angleRoll * 0.87 + angleRoll_acc * 0.13;
  }
  else
  {
    anglePitch = anglePitch_acc;
    angleRoll = angleRoll_acc;
    setGyro_angles = true;
  }

  // nilai angle pitch dan roll yang difilter menggunakan complementary filter
  anglePitch_output = anglePitch_output * 0.9 + anglePitch * 0.1;
  angleRoll_output = angleRoll_output * 0.9 + angleRoll * 0.1;

  while (micros() - loopTimer < Ts*10000000);
  loopTimer = micros();
}

void YawPitch()
{
  // Convert nilai radian output MPU6050 ke derajat nilai servo
  // Yaw   = anglePitch_output * 180 / 3.1415;
  // Pitch = angleRoll_output * 180 / 3.1415;
  Yaw = anglePitch_output;
  Pitch = angleRoll_output;
    Yaw = Yaw - correct; // Set yaw to 0 deg - substract dengan nilai akhir random yaw
    // Mapping nilai -90 ~ 90 ke 0 ~ 180 (merubah nilai radian to deggre)
    int servoYawValue   = map((int)Yaw, -90, 90 , 0 , 180);
    int servoPitchValue = map((int)Pitch, -90, 90 , 0 , 180);
    ServoYaw = servoYawValue;
    ServoPitch = servoPitchValue;


}

void lcdOutput()
{
  delay(1);
  lcd.setCursor(0,0);
  lcd.print(ServoYaw);
  lcd.setCursor(0,1);
  lcd.print(ServoPitch);
}