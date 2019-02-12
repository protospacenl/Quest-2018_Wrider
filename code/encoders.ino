/*
 * This file includes the encoder interrupt functions. All encoders are head on their rising edge. 
 * If the given motor is initialising it's position the encoder does not look at the direction but only counts up so the value can be compared in the initMotor function.
 * after this the encoder looks at the direction that is set when moving the motor. If the motoor is moving right it counts up, if it's moving left it counts down. 
 * When the motor has reached its positon or tries to go over it's maximum value the motor is stopped with the functions stopMotor();
 * the Yaw and Elbow are a special case because the yaw moves with the elbow with a factor diffrence of 3. The yaw can also be set to follow the elbow by making yawFollowElbow true. 
 */
void readShoulderEncoder()
{
  if(initShoulder == 1)
    {
        shoulderEncoder++;
    }
    #ifdef MAXIMUM_LIMIT
    else if(shoulderEncoder >= SHOULDER_MAX_POS)
      stopMotor(SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE);
    #endif
    //if the encoder counts matches the target count, the motor will be disabled
    else if(shoulderEncoder == shoulderTarget && !basicMovement)
    {
        stopMotor(SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE);
        #ifdef DEBUG
       // Serial.println("Shoulder Pos reached");
        #endif
    }

    //based of the given direction the encoder count increments or decrements.
    else if(shoulderDirection == RIGHT)
    {
        shoulderEncoder++;
    }
    else if(shoulderDirection == LEFT)
    {
          #ifdef MAXIMUM_LIMIT
          if(shoulderEncoder <= 0)
          {
            stopMotor(SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE);
          }
          #endif
          shoulderEncoder--;
    }
   else
     Serial.println("Something in shoudler encoder went wrong");
}

void readElbowEncoder()
{
   if(initElbow == 1)
    {
        elbowEncoder++;
        //static int yawCount = 0;
        moveToPos(&yawTarget,(YAW_MAX_POS/2) + yawCounter , &yawEncoder, YAW_PLUS, YAW_MIN, YAW_ENABLE, 60);
        if(elbowEncoder % 3 == 0)
        {
          yawCounter++;
        }
    }
    #ifdef MAXIMUM_LIMIT
    else if(elbowEncoder >= ELBOW_MAX_POS)
      stopMotor(ELBOW_PLUS, ELBOW_MIN, ELBOW_ENABLE);
    #endif
    else if(elbowEncoder == elbowTarget && !basicMovement)
    {
      stopMotor(ELBOW_PLUS, ELBOW_MIN, ELBOW_ENABLE);
      #ifdef DEBUG
      // Serial.println("ELBOW Pos reached");
      #endif
    }
    else if(elbowDirection == RIGHT)
    {
      #ifdef TEST_YAW
       elbowEncoder++;
        #ifdef FOLLOW_YAW
         if(yawFollowElbow == true)
          moveToPos(&yawTarget,(YAW_MAX_POS/2) + yawCounter , &yawEncoder, YAW_PLUS, YAW_MIN, YAW_ENABLE, 60);
        #endif
         if(elbowEncoder %3 == 0)
           yawCounter++;
      #else
      elbowEncoder++;
      if(yawMiddle == true)
          moveToPos(&yawTarget,(YAW_MAX_POS/2) + yawCounter , &yawEncoder, YAW_PLUS, YAW_MIN, YAW_ENABLE, 60);
      if(yawEncoderCounter %18 == 0 && initElbow == 0)//maybe this needs to be six //15 works kinda //if()
        yawEncoder--;//this sucks
      yawEncoderCounter++;

      #endif
    }
    else if(elbowDirection == LEFT)
    {
      #ifdef TEST_YAW
      elbowEncoder--;
        #ifdef FOLLOW_YAW
        if(yawFollowElbow == true)
        #endif
         moveToPos(&yawTarget,(YAW_MAX_POS/2) + yawCounter , &yawEncoder, YAW_PLUS, YAW_MIN, YAW_ENABLE, 60);
        if(elbowEncoder % 3 == 0)
          yawCounter--;
      #else
      if(elbowEncoder > 0)
        elbowEncoder--;
        if(yawMiddle == true)
            moveToPos(&yawTarget,(YAW_MAX_POS/2) + yawCounter , &yawEncoder, YAW_PLUS, YAW_MIN, YAW_ENABLE, 60);
      if(yawEncoderCounter %18== 0 && initElbow == 0)//maybe this needs to be six //15 works kinda
        yawEncoder++;
      yawEncoderCounter++;
      #endif
    }
   else
     Serial.println("Something in elbow encoder went wrong");
}
void readWristRHEncoder()
{
   if(initWristRH == 1)
    {
      wristRHEncoder++;
    }
    else if(wristRHEncoder == wristRHTarget)
    {
      stopMotor(WRIST_RH_PLUS, WRIST_RH_MIN, WRIST_RH_ENABLE);
      #ifdef DEBUG
        // Serial.println("wristPos reached");
      #endif
    }
    else if(wristRHDirection == RIGHT)
    {
      wristRHEncoder++;
    }
    else if(wristRHDirection == LEFT)
    {
      if(wristRHEncoder > 0)
        wristRHEncoder--;
    }
    else
      Serial.println("Something in wrist encoder went wrong");
}
void readWristLHEncoder()
{
  if(initWristLH == 1)
    {
      wristLHEncoder++;
    }
    else if(wristLHEncoder == wristLHTarget)
    {
      stopMotor(WRIST_LH_PLUS, WRIST_LH_MIN, WRIST_LH_ENABLE);
      #ifdef DEBUG
        // Serial.println("wristLH Pos reached");
      #endif
    }
    else if(wristLHDirection == RIGHT)
    {
      wristLHEncoder++;
    }
    else if(wristLHDirection == LEFT)
    {
      if(wristLHEncoder > 0)
        wristLHEncoder--;
    }
    else
      Serial.println("Something in wristLH encoder went wrong");
}
void readZedEncoder()
{
  if(initZed == 1)
    {
      zedEncoder++;
    }
    #ifdef MIXIMUM_LIMIT
    else if(zenEncoder >= ZED_MAX_POS)
      stopMotor(ZED_PLUS, ZED_MIN, ZED_ENABLE);
    #endif
    else if(zedEncoder == zedTarget && !basicMovement)
    {
      stopMotor(ZED_PLUS, ZED_MIN, ZED_ENABLE);
      #ifdef DEBUG
      //Serial.println("zed Pos reached");
      #endif
    }
    else if(zedDirection == RIGHT)
    {
      zedEncoder++;
    }
    else if(zedDirection == LEFT)
    {
      if(zedEncoder > 0)
        zedEncoder--;
    }
    else
      Serial.println("Something in zed encoder went wrong");
      Serial.println("ZED encoder");
}
void readYawEncoder()
{
 if(initYaw == 1)
    {
      yawEncoder++;
    }
    #ifdef MAXIMUM_LIMIT
    else if(yawEncoder >= YAW_MAX_POS)
      stopMotor(YAW_PLUS, YAW_MIN, YAW_ENABLE);
    #endif
    else if(yawEncoder == yawTarget && !basicMovement)
    {
      stopMotor(YAW_PLUS, YAW_MIN, YAW_ENABLE);
      #ifdef DEBUG
        // Serial.println("yaw Pos reached");
      #endif
    }
    else if(yawDirection == RIGHT )
    {
      yawEncoder++;
    }
    else if(yawDirection == LEFT )
    {
      if(yawEncoder > 0)
        yawEncoder--;
    }
    else
      Serial.println("Something in yaw encoder went wrong");
}

