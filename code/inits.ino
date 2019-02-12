/*
 * This file includes the initialisation of all the motors. In the main file you can set the difines: 
 * WRIST, YAW, SHOULDER, ELBOW, YAW on or off so you can choose what motor you want to use or test.
 */
void initAllMotors()
{
  //init all the motors;
  #ifdef WRIST
      initWrist(&wristLHEncoder, &wristRHEncoder);
      moveToPos(&wristLHTarget, 0, &wristLHEncoder, WRIST_LH_PLUS, WRIST_LH_MIN, WRIST_LH_ENABLE, 235);
      moveToPos(&wristRHTarget, 0, &wristRHEncoder, WRIST_RH_PLUS, WRIST_RH_MIN, WRIST_RH_ENABLE, 250);
  #endif
  #ifdef WRIST_RH
  //init wrist
      initMotor(&initWristRH, &wristRHEncoder, WRIST_RH_PLUS, WRIST_RH_MIN, WRIST_RH_ENABLE, LEFT, 200);
  #endif
  #ifdef WRIST_LH
  //inits wrist LH
      initMotor(&initWristLH, &wristLHEncoder, WRIST_LH_PLUS, WRIST_LH_MIN, WRIST_LH_ENABLE, LEFT, 200);
  #endif
  #ifdef YAW
  //inits YAW
      initMotor(&initYaw, &yawEncoder, YAW_PLUS, YAW_MIN, YAW_ENABLE, LEFT, 70);
      moveToPos(&yawTarget, YAW_MAX_POS/2, &yawEncoder, YAW_PLUS, YAW_MIN, YAW_ENABLE, 90);//100
      delay(400);
  #endif
  //inits shoulder
  #ifdef SHOULDER
      initMotor(&initShoulder, &shoulderEncoder, SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE, LEFT, 135);//150//135
      delay(900);
  #endif
  #ifdef ELBOW
  //init elbow
      initMotor(&initElbow, &elbowEncoder, ELBOW_PLUS, ELBOW_MIN, ELBOW_ENABLE, RIGHT, 150);//150
      moveToPos(&shoulderTarget, SHOULDER_MAX_POS/2, &shoulderEncoder, SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE, 180);//180
      moveToPos(&elbowTarget, ELBOW_MAX_POS/2 , &elbowEncoder, ELBOW_PLUS, ELBOW_MIN ,ELBOW_ENABLE, 120);//120
      moveToPos(&yawTarget, (YAW_MAX_POS/2), &yawEncoder, YAW_PLUS, YAW_MIN, YAW_ENABLE, 65);
      delay(4200);
  #endif
  #ifdef ZED
  //inits ZED
      initMotor(&initZed, &zedEncoder, ZED_PLUS, ZED_MIN, ZED_ENABLE, LEFT, 240);
      moveToPos(&zedTarget, 400, &zedEncoder, ZED_PLUS, ZED_MIN, ZED_ENABLE, 250);
      delay(3000);
  #endif
  #ifdef DEBUG
      Serial.println("all setups complete");
  #endif
}

//inits the provided motor by moving it till the encoders stop counting.
void initMotor(int *initVar, volatile int *encoder, int motorPlus, int minus, int enable, int moveDirection, int moveSpeed)
{
  int tempEncoderCount = -1;
  *initVar = 1;
  #ifdef DEBUG
  Serial.println(initShoulder);
  #endif
  while(*encoder != tempEncoderCount/*getEncoderCount(motorPlus) != tempEncoderCount*/)
    {
      tempEncoderCount = *encoder;//tempEncoderCount = getEncoderCount(motorPlus);
      moveMotor(100, moveSpeed, moveDirection, motorPlus, minus, enable);
    }
    if(moveDirection == RIGHT)
      {
      switch(motorPlus)
      {
        case SHOULDER_PLUS:
          *encoder = SHOULDER_MAX_POS;
          break;
         case ELBOW_PLUS:
           *encoder = ELBOW_MAX_POS;
            break;
         default:
            Serial.println("init switch error");
            break; 
       }
      }
      else
          *encoder = 0;
    #ifdef DEBUG
        Serial.println(initShoulder);
    #endif
  *initVar = 0;
  
  //prints what motor setup is complete
  switch(motorPlus)
    {
      case(SHOULDER_PLUS):
          Serial.println("Shoulder setup complete");
          break;
     case(ELBOW_PLUS):
          Serial.println("elbow setup complete");
          break;
     case(WRIST_RH_PLUS):
          Serial.println("wrist setup complete");
          break;
     case(WRIST_LH_PLUS):
          Serial.println("Wrist LH setup complete");
          break;
     case(ZED_PLUS):
          Serial.println("Zed setup complete");
          break;
     case(YAW_PLUS):
          Serial.println("yaw setup complete");
          break;
     default:
          Serial.println("init went very wrong");
          break;
    }
}
void initWrist(volatile int *encoderLH, volatile int *encoderRH)
{
  int tempLHCount = -1;
  int tempRHCount = -1;
  initWristLH = 1;
  initWristRH = 1;
  Serial.print("init LH = ");
  Serial.println(initWristLH);
  while(*encoderLH != tempLHCount && *encoderRH != tempRHCount)
  {
      analogWrite(WRIST_LH_ENABLE, 255);
      digitalWrite(WRIST_LH_PLUS, LOW);
      digitalWrite(WRIST_LH_MIN, HIGH);
      analogWrite(WRIST_RH_ENABLE, 255);
      digitalWrite(WRIST_RH_PLUS, HIGH);
      digitalWrite(WRIST_RH_MIN, LOW);
      tempLHCount = *encoderLH;
      tempRHCount = *encoderRH;
      delay(100);
      if(tempLHCount >= WRIST_LH_MAX_POS * 2)
      {
          Serial.println("breaking up");
          break;
      }
  }
  stopMotor(WRIST_LH_PLUS, WRIST_LH_MIN, WRIST_LH_ENABLE);
  stopMotor(WRIST_RH_PLUS, WRIST_LH_MIN, WRIST_RH_ENABLE);
  tempLHCount = -1;
  tempRHCount = -1;
  *encoderLH = 0;
  *encoderRH = 0;
  Serial.println("up done");
  delay(400);
  while(*encoderLH != tempLHCount && *encoderRH != tempRHCount)
  {
    analogWrite(WRIST_LH_ENABLE, 255);
    digitalWrite(WRIST_LH_PLUS,LOW);
    digitalWrite(WRIST_LH_MIN, HIGH);
    analogWrite(WRIST_RH_ENABLE, 255);
    digitalWrite(WRIST_RH_PLUS, LOW);
    digitalWrite(WRIST_RH_MIN, HIGH);
    tempLHCount = *encoderLH;
    tempRHCount = *encoderRH;
    delay(100);
    if(tempRHCount >= WRIST_LH_MAX_POS * 2)
    {
        Serial.println("breaking rotation");
        break;
    }
  }
  stopMotor(WRIST_LH_PLUS, WRIST_LH_MIN, WRIST_LH_ENABLE);
  stopMotor(WRIST_RH_PLUS, WRIST_LH_MIN, WRIST_RH_ENABLE);
  *encoderLH = -1222;
  *encoderRH = -1222;
  initWristLH = 0;
  initWristRH = 0;
}
