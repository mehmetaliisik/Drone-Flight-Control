//RampBreakers Receiver Code
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <MPU9250_WE.h>
#include <Wire.h>

//Motorların Tanımı
Servo FrontLeftMotor;
Servo FrontRightMotor;
Servo BackLeftMotor;
Servo BackRightMotor;

MPU9250_WE myMPU9250 = MPU9250_WE(0x68); //IMU tanımı

//Pitch ve Rol değerleri
#define KProllPitch 0.2
#define KIrollPitch 0.0005
#define KDrollPitch 1.9

//Yaw değerleri
#define KPyaw 0.40
#define KIyaw 0.50
#define KDyaw 0.00

RF24 radio(9, 10); // CE, CSN pinleri
const byte address[13] = "rampBreakers"; //Haberleşme Kodu

//Değişkenlerin tanımlanması
unsigned long timee, prevTime, deltaTime; // Zaman Değişkenleri
double roll_pid_i, roll_last_error, pitch_pid_i, pitch_last_error, yaw_pid_i, yaw_last_error;
double roll_control_signal, pitch_control_signal, yaw_control_signal;
int valueForGas;// Kumandadan gelen gaz değeri
int frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor, throttle; //Motor pwm değerleri
double valueForRoll, valueForPitch, valueForYaw;
//double currentYawAngle, previousYawAngle;

void mpu9250()//IMU Kalibrasyon
{
  Wire.begin();
  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  myMPU9250.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);
  myMPU9250.setGyrOffsets(45.0, 135.0, -105.0);
  Serial.println("Done!");
  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);
  myMPU9250.setSampleRateDivider(5);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  delay(200);
}

void setup() 
{
  Serial.begin(115200);
  radio.begin(); // Haberleşme başlatıldı
  radio.openReadingPipe(0, address); //Gelen  sinyaldeki adresi kontrol ediyor
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  mpu9250(); //IMU Kalibrasyon
  FrontLeftMotor.attach(4,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  FrontRightMotor.attach(5,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
  BackLeftMotor.attach(6,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  BackRightMotor.attach(2,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  stopMotors();
  delay(2000);
}

void loop() {
  if (radio.available()) {
    Serial.println("--- RampBreakers ---"); //Haberleşmenin çalıştığını seri monitörde görüyoruz
    int remoteValue[4]; //Gelecek değerler için array tanımlandı 
    radio.read(remoteValue, sizeof(remoteValue)); //Diğer haberleşmeden veri alındı
    
    timee = millis(); //Zaman hesaplanmaya başlandı
    prevTime = timee; //Ölçülen zaman başka değere atandı

    //Kumandadan gelen değerler değişkenlere atandı             ????????????
    valueForGas = map(remoteValue[0], 0, 1023, 0, 180);
    valueForYaw = map(remoteValue[1], 0, 1023, -90, 90);
    valueForRoll = map(remoteValue[2], 0, 1023, -90, 90);
    valueForPitch = map(remoteValue[3], 0, 1023, -90, 90); 

    //Sınır çizgileri belirlendi                               ?????????????
    valueForGas = constrain(valueForGas, 0, 1023);
    valueForRoll = constrain(valueForRoll, -90, 90);
    valueForPitch = constrain(valueForPitch, -90, 90);
    valueForYaw = constrain(valueForYaw, -90, 90);
 
    calculateMotorPowers(); //Motor güçleri hesaplandı
    
    if(valueForGas < 30) //Gaz 30 altına düşerse motorlar dursun
    {
      stopMotors();
      Serial.print("Gas: ");
      Serial.println(throttle);
    }
    else //Throttle 30'den büyükse motorlara güç verilsin
    {
      FrontLeftMotor.write(frontLeftMotor);
      FrontRightMotor.write(frontRightMotor);
      BackLeftMotor.write(backLeftMotor);
      BackRightMotor.write(backRightMotor);
      Serial.print("1: ");
      Serial.println(frontLeftMotor);
      Serial.print("2: ");
      Serial.println(frontRightMotor);
      Serial.print("3: ");
      Serial.println(backLeftMotor);
      Serial.print("4: ");
      Serial.println(backRightMotor);
      delay(200);
   }                                                                                                                                                                       
  }
  else
  {
    Serial.println("!!There is an error!!"); //Eğer haberleşmede hata varsa yazdıracak
  }
}
