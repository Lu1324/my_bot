/* Serial port baud rate */
#define BAUDRATE     57600
/* Maximum PWM signal */
#define MAX_PWM        200

#define PID_RATE           15    // Hz

#define POINTS      100

bool rev_l = false;
bool rev_r = false;

#include "commands.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "diff_controller.h"
/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;
/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;
#define AUTO_STOP_INTERVAL 1500
long lastMotorCommand = AUTO_STOP_INTERVAL;


/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int y = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  if (arg1<0)  rev_l = true;
  else if (arg1>0) rev_l = false;

  arg2 = atoi(argv2);
  if (arg2 < 0) rev_r = true;
  else if (arg2 > 0) rev_r = false;
  
  switch(cmd) {
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;
    case ANALOG_READ:
      Serial.println(analogRead(arg1));
      break;
    case DIGITAL_READ:
      Serial.println(digitalRead(arg1));
      break;
    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK"); 
      break;
    case DIGITAL_WRITE:
      if (arg2 == 0) digitalWrite(arg1, LOW);
      else if (arg2 == 1) digitalWrite(arg1, HIGH);
      Serial.println("OK"); 
      break;
    case PIN_MODE:
      if (arg2 == 0) pinMode(arg1, INPUT);
      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      Serial.println("OK");
      break;
    case READ_ENCODERS:
      Serial.print(readEncoder(LEFT));
      Serial.print(" ");
      Serial.println(readEncoder(RIGHT));
      break;
    #ifdef DE
    case DEBUG:
      Serial.print("enc left: ");
      for(int x = 0; x<POINTS; ++x) {
        Serial.print(String(encl[x]) + " ");
      }
      Serial.println();
          Serial.print("enc right: ");
      for(int x = 0; x<POINTS; ++x) {
        Serial.print(String(encr[x]) + " ");
      }
      Serial.println();
          Serial.print("output left: ");
      for(int x = 0; x<POINTS; ++x) {
        Serial.print(String(outl[x]) + " ");
      }
      Serial.println();
          Serial.print("output right: ");
      for(int x = 0; x<POINTS; ++x) {
        Serial.print(String(outr[x]) + " ");
      }
      Serial.println();
      break;
    #endif
    case RESET_ENCODERS:
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      if (arg1 == 0 && arg2 == 0) {
        setMotorSpeeds(0, 0);
        resetPID();
        moving = 0;
      }
      else moving = 1;
      // TODO. adapt new speed sceme
      leftPID.TargetTicksPerFrame = arg1;
      rightPID.TargetTicksPerFrame = arg2;
      Serial.println("OK"); 
      break;
    case MOTOR_RAW_PWM:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      resetPID();
      moving = 0; // Sneaky way to temporarily disable the PID
      setMotorSpeeds(arg1, arg2);
      Serial.println("OK"); 
      break;
    case PRINT_PID:
      for(int x = 0; x<4; ++x) {
        Serial.print(String(Kd) + " " + String(Kp) + " " + String(Kd) + " " + String(Ki) + " " + String(Ko) + " ");
      }
      Serial.println();
      break;
    case UPDATE_PID:
      while ((str = strtok_r(p, ":", &p)) != '\0') {
        pid_args[y] = atoi(str);
        y++;
      }
      Kp = pid_args[0];
      Kd = pid_args[1];
      Ki = pid_args[2];
      Ko = pid_args[3];
      Serial.println("OK");
      break;
    default:
      Serial.println("Invalid Command");
      break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);
  myEncCounterSetup();
  initMotorController();
  resetPID();
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
  
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    setMotorSpeeds(0, 0);
    moving = 0;
  }
}

