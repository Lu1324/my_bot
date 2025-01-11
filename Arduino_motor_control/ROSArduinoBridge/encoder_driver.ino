/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */

long counter_left = 0;
long counter_right = 0;

unsigned long lastInterrupt_l;
unsigned long lastInterrupt_r;

void myEncCounterSetup() {
  pinMode(LEFT_ENC_PIN, INPUT_PULLUP); // Setzen des analogen Pins als Input
  pinMode(RIGHT_ENC_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN), count_left, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN), count_right, RISING);
}

void count_left(){
  if(millis() - lastInterrupt_l > 10) {    
    if (rev_l) {
      --counter_left;
    }
    else ++counter_left;
    lastInterrupt_l = millis();
  }
}
void count_right(){
  if(millis() - lastInterrupt_r > 10) {    
    if(rev_r) {
      --counter_right;
    }
    else	++counter_right;
    lastInterrupt_r = millis();
    }
}


long readEncoder(int i) {
  if(i == LEFT)
    return counter_left;
  else
    return counter_right;
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
  if (i == LEFT){
    counter_left=0;
    return;
  } else { 
    counter_right=0;
    return;
  }
}

void resetEncoders() {
  counter_left=0;
  counter_right=0;
  //i=0;
  return;
}

