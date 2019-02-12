//handles the communication.

void readData()
{
  if(Serial.available()> 0)
  {
      char incommingData = Serial.read();
      Serial.println(incommingData);
      switch(incommingData)
      {
        case 's':
          Serial.println("Stopping all motors");
          stopAllMotors();
          break;
        case 'a':
          Serial.println("resetting position");
          toStartPos();
          break;
        case 'r':
          #ifdef BASIC_MOVEMENT
          basicRight();
          #else
          moveRight();
          #endif
          Serial.println("moving Right");
          break;
        case 'l':
          #ifdef BASIC_MOVEMENT
          basicLeft();
          #else
          //moveLeft();
          #endif
          Serial.println("moving left");
          break;
        case 'u':
          moveUp();
          Serial.println("moving up");
          break;
        case 'd':
          moveDown();
          Serial.println("moving down");
          break;
        case 'f':
          moveForward();
          Serial.println("moving forward");
          break;
        case 'b':
          moveBackwards();
          Serial.println("moving backwards");
          break;
       case 'z':
          Serial.println(zedEncoder);
           break;
       case 'x':
          Serial.println(shoulderEncoder);
          break;
       case 'c': 
          Serial.println(elbowEncoder);
          break;
       case 'v':
          Serial.println(yawEncoder);
          break;
       case 't':
          Serial.println(shoulderDirection);
          break;
       case 'I':
          Serial.println("initialising Robot");
          initAllMotors();
       default:
          Serial.println("invalid command");
      }
  }
}
