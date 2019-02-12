/*
 * this file includes basic movement such as: up/down, left/right, forward/backwards. These functions are used by the tracking software to controll the position of the motor.
 */
void stopAllMotors()
{
  for(int i =22; i < 35; i++)
      digitalWrite(i, LOW);
  for(int j = 4; j < 10; j++)
      analogWrite(j, 0);
  Serial.println("Stopping all motors");
}
//stops the targeted motor
void stopMotor(int plus, int minus, int enable)
{
  switch(plus)
  {
     case SHOULDER_PLUS:
        shoulderDirection = STOP;  
        break;
     case ELBOW_PLUS:
        elbowDirection = STOP;  
        break;
     case WRIST_RH_PLUS:
        wristRHDirection = STOP;  
        break;
     case WRIST_LH_PLUS:
        wristLHDirection = STOP;  
        break;
     case ZED_PLUS:
        zedDirection = STOP;  
        break;
     case YAW_PLUS:
        yawDirection = STOP;  
        break;
     default:
        Serial.println("error on direction");
        break;
   if(initShoulder != 1)
    Serial.println("Stopping motor");
   }
  digitalWrite(plus, LOW);
  digitalWrite(minus, LOW);
  digitalWrite(enable, LOW);
  basicMovement = false;
  //Serial.println("Stopping motor");
}
void motorRight(int plus, int minus, int enable, int moveSpeed)
{
       setDirection(plus, RIGHT);
       analogWrite(enable, moveSpeed);
       digitalWrite(plus,  HIGH);
       digitalWrite(minus, LOW);
       basicMovement = true;
}
void motorLeft(int plus, int minus, int enable, int moveSpeed)
{
      setDirection(plus, LEFT);
      analogWrite(enable, moveSpeed);
      digitalWrite(plus,  LOW);
      digitalWrite(minus, HIGH);
      basicMovement = true;
}
void moveRight()
{
    motorRight(SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE, 180);
  motorRight(ELBOW_PLUS, ELBOW_MIN, ELBOW_ENABLE, 120);
  //yawFollowElbow = true; 
  yawMiddle = true;
  basicMovement = true;
}

void moveLeft()
{
  motorLeft(SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE, 180);
  motorLeft(ELBOW_PLUS, ELBOW_MIN, ELBOW_ENABLE, 120);
  //yawFollowElbow = true; 
  yawMiddle = true;
}

void moveForward()
{
  motorRight(SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE, 180);
  motorLeft(ELBOW_PLUS, ELBOW_MIN, ELBOW_ENABLE, 120);
  //yawFollowElbow = true;
  yawMiddle = true;
  //movingForward = true;
}

void moveBackwards()
{
  motorLeft(SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE, 180);
  motorRight(ELBOW_PLUS, ELBOW_MIN, ELBOW_ENABLE, 120);
  //yawFollowElbow = true; 
  yawMiddle = true;
}
void toStartPos()
{
      moveToPos(&shoulderTarget, SHOULDER_MAX_POS/2, &shoulderEncoder, SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE, 180);//180
      moveToPos(&elbowTarget, ELBOW_MAX_POS/2 , &elbowEncoder, ELBOW_PLUS, ELBOW_MIN ,ELBOW_ENABLE, 120);//120
      moveToPos(&yawTarget, (YAW_MAX_POS/2), &yawEncoder, YAW_PLUS, YAW_MIN, YAW_ENABLE, 65);
      moveToPos(&zedTarget, ZED_MAX_POS/2, &zedEncoder, ZED_PLUS, ZED_MIN, ZED_ENABLE, 250);
      while(elbowDirection != STOP && shoulderDirection != STOP && zedDirection != STOP && yawDirection != STOP)
        delay(100);
}
void moveUp()
{
  motorRight(ZED_PLUS, ZED_MIN, ZED_ENABLE, 250);
  basicMovement = true;
}
void moveDown()
{
  motorLeft(ZED_PLUS, ZED_MIN, ZED_ENABLE, 250);
  basicMovement = true;
}
void basicLeft()
{
  motorLeft(YAW_PLUS, YAW_MIN, YAW_ENABLE, 65);
  basicMovement = true;
}

void basicRight()
{
  motorRight(YAW_PLUS, YAW_MIN, YAW_ENABLE, 65);
  basicMovement = true;
}
