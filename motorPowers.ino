void stopMotors()
{
  FrontLeftMotor.write(0); //Motorlar sıfırlandı
  FrontRightMotor.write(0);
  BackLeftMotor.write(0);
  BackRightMotor.write(0);
  roll_pid_i = 0;
  roll_last_error = 0;
  pitch_pid_i = 0;
  pitch_last_error = 0;
  yaw_pid_i = 0;
  yaw_last_error = 0;
}

double getControlSignal(double error, double kp, double ki, double kd, double& pid_i,double& last_error, unsigned long delta_time) //Kontrol sinyalleri hesaplandı
{
  double pid_p = error;
  double pid_d = (error - last_error) / delta_time; //Hatanın türevi alındı
  pid_i += error * delta_time; //Hatanın integrali alındı
  double control_signal = (kp*pid_p) + (ki*pid_i) + (kd*pid_d); //PID hesabı yapoldı
  last_error = error;
  return control_signal;
}

void calculateMotorPowers() 
{
  xyzFloat accCorrRaw = myMPU9250.getCorrectedAccRawValues(); //İvme değerleri IMU'dan çekildi
  double yaw = 180 * atan (accCorrRaw.y/sqrt(accCorrRaw.x*accCorrRaw.x + accCorrRaw.y*accCorrRaw.y))/M_PI; //Yaw değeri hesabı M_PI => pi değeri
  //Hatalar hesaplandı
  double rollError = valueForRoll - myMPU9250.getRoll()+2; 
  double pitchError = valueForPitch - myMPU9250.getPitch()+2;
  double yawError = valueForYaw - yaw;
  
  //Zaman hesabı
  timee = millis();
  deltaTime = timee - prevTime;

  //Kontrol sinyallerinin hesabı
  roll_control_signal = getControlSignal(rollError, KProllPitch, KIrollPitch, KDrollPitch, roll_pid_i, roll_last_error, deltaTime);
  pitch_control_signal = getControlSignal(pitchError, KProllPitch, KIrollPitch, KDrollPitch, pitch_pid_i, pitch_last_error, deltaTime);
  yaw_control_signal = getControlSignal(yawError, KPyaw, KIyaw, KDyaw, yaw_pid_i, yaw_last_error, deltaTime);
  
  //Motorlar için güç hesabı yapıldı
  frontLeftMotor = round(valueForGas + roll_control_signal + pitch_control_signal - yaw_control_signal);
  frontRightMotor = round(valueForGas - roll_control_signal + pitch_control_signal + yaw_control_signal);
  backLeftMotor = round(valueForGas + roll_control_signal - pitch_control_signal + yaw_control_signal);
  backRightMotor = round(valueForGas - roll_control_signal - pitch_control_signal - yaw_control_signal);
  
  int maxMotorPower = max(max(frontLeftMotor, frontRightMotor), max(backLeftMotor, backRightMotor)); //Max motor güçleri hesaplandı 
  if (maxMotorPower > 180) //Max motor gücü 180'den yüksekse 180'e düşürsün
  { 
    double powerReductionRate = (double)maxMotorPower / (double)180;
    frontLeftMotor = round((double)frontLeftMotor / powerReductionRate);
    frontRightMotor = round((double)frontRightMotor / powerReductionRate);
    backLeftMotor = round((double)backLeftMotor / powerReductionRate);
    backRightMotor = round((double)backRightMotor / powerReductionRate);
    throttle = frontLeftMotor + frontRightMotor + backLeftMotor + backRightMotor;
  }
}
