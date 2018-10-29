
#include "Adafruit_seesaw.h"

#include <Wire.h>
#include <Adafruit_MotorShield.h>


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *M1 = AFMS.getMotor(1);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);


#define ENC_A (7)
#define ENC_B (11)
#define DIRECTION (1)

int prevEncs;
int32_t qdecPos;

const int8_t quadratureTruthTable[4][4] = {{ 0, 1,-1, 0},
                                           {-1, 0, 0, 1},
                                           { 1, 0, 0,-1},
                                           { 0,-1, 1, 0}};

void quadrature()
{
  int currEncs = (digitalRead(ENC_B) << 1) + (digitalRead(ENC_A));
  qdecPos += DIRECTION * quadratureTruthTable[prevEncs][currEncs];

  prevEncs = currEncs;
}


#define CONTROL_LOOP_TIME_uS (5000)
unsigned long prevMicros;


float kp;
float ki;
float kd;

#define INTEGRAL_LIMIT (2000)
#define EFFORT_LIMIT (10000)

float integralError;
float prevPosition;
float prevDesired;

int tNow;


/*****************GENERIC FUNCTIONS***********************************/

float clamp(float input, float maxVal, float minVal)
{
  return min(maxVal, max(minVal, input));
}

/*****************GENERIC FUNCTIONS***********************************/


/*****************MOTOR CONTROL FUNCTIONS***********************************/
void motorInit(Adafruit_DCMotor* motor)
{
  // Set the speed to start, from 0 (off) to 255 (max speed)
  motor->setSpeed(0);

  // turn on motor
  motor->run(RELEASE);

  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);

  qdecPos = 0;
  prevEncs = 0;

  attachInterrupt(digitalPinToInterrupt(ENC_A), quadrature, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), quadrature, CHANGE);
  
  integralError = 0;
  prevPosition = 0;
  prevDesired = 0;
  tNow = 0;
  
  kp = 10;
  ki = 0.4;
  kd = 10;
}

void motorControl(Adafruit_DCMotor* motor, float thisDesiredPosition)
{
  float positionError, velocity, desiredVelocity, velocityError, effort;

  positionError = thisDesiredPosition - ((float)qdecPos);

  desiredVelocity = thisDesiredPosition - prevDesired;
  velocity = qdecPos - prevPosition;
  velocityError = desiredVelocity - velocity;
  
  integralError = clamp(integralError + positionError, INTEGRAL_LIMIT, -INTEGRAL_LIMIT);

  prevPosition = qdecPos;
  prevDesired = thisDesiredPosition;

  effort = 0;
  effort += kp*positionError;
  effort += ki*integralError;
  effort += kd*velocityError;

  effort = clamp(effort, EFFORT_LIMIT, -EFFORT_LIMIT);

  //converts effort to 254 scale pwm effort
  effort = (254.0/EFFORT_LIMIT) * effort;

  if(effort < 10 && effort > -10)
  {
    effort = 0;
  }

//  Serial.println(velErr);
  if(effort > 0)
  {
    //FORWARD
    motor->setSpeed(effort);
    motor->run(FORWARD);
  }
  else if(effort < 0)
  {
    //BACKWARD
    effort = -1.0*effort;
    motor->setSpeed(effort);
    motor->run(BACKWARD);
  }
  else
  {
    effort = 0;
    motor->setSpeed(effort);
    motor->run(RELEASE);
  }
  // Serial.print(qdecPos);
  // Serial.print(" ");
  // Serial.println(thisDesiredPosition);

//  Serial.println(positionError);
}

/*****************MOTOR CONTROL FUNCTIONS***********************************/

/*****************JOYSTICK FUNCTIONS***********************************/

Adafruit_seesaw ss;

#define BUTTON_RIGHT 6
#define BUTTON_DOWN  7
#define BUTTON_LEFT  9
#define BUTTON_UP    10
#define BUTTON_SEL   14
uint32_t button_mask = (1 << BUTTON_RIGHT) | (1 << BUTTON_DOWN) | 
                (1 << BUTTON_LEFT) | (1 << BUTTON_UP) | (1 << BUTTON_SEL);

#if defined(ESP8266)
  #define IRQ_PIN   2
#elif defined(ESP32)
  #define IRQ_PIN   14
#elif defined(NRF52)
  #define IRQ_PIN   27
#elif defined(TEENSYDUINO)
  #define IRQ_PIN   8
#elif defined(ARDUINO_ARCH_WICED)
  #define IRQ_PIN   PC5
#else
  #define IRQ_PIN   5
#endif

int joyX, joyY, prevJoyX, prevJoyY;
uint32_t buttons;

#define DEBOUNCE (500)
bool buttonPressed;
unsigned long buttonBTimePressed;

void ssInit()
{
  if(!ss.begin(0x49)){
    Serial.println("ERROR! seesaw not found");
//    while(1);
  }
  else 
  {
    Serial.println("seesaw started");
    Serial.print("version: ");
    Serial.println(ss.getVersion(), HEX);
  }
  ss.pinModeBulk(button_mask, INPUT_PULLUP);
  ss.setGPIOInterrupts(button_mask, 1);

  pinMode(IRQ_PIN, INPUT);
}

void readJoyWing()
{
  joyX = ss.analogRead(3);
  joyY = ss.analogRead(2);
  buttons = ss.digitalReadBulk(button_mask);
}

bool isBPressed()
{
  return (! (buttons & (1 << BUTTON_DOWN)));
}


/*****************JOYSTICK FUNCTIONS***********************************/



/***************************************************************************/


// #define FULL_HEAD_TURN_COUNTS (20000)

#define FULL_HEAD_TURN_COUNTS (4200)
#define RANDOM_TIME_LIMIT (90000)

typedef enum
{
  om_startUp,
  om_manual,
  om_autoCycle,
  om_autoRandom,
  om_btManual,
  om_numModes
} OperationMode_t;
OperationMode_t operationMode;


void incrementMode()
{
  switch(operationMode)
  {
    case om_startUp:
    operationMode = om_manual;
    break;

    case om_manual:
    operationMode = om_autoCycle;
    break;

    case om_autoCycle:
    operationMode = om_autoRandom;
    break;

    case om_autoRandom:
    operationMode = om_btManual;
    break;

    case om_btManual:
    operationMode = om_numModes;
    break;

    case om_numModes:
    operationMode = om_startUp;
    break;
  }
}



float desiredPosition;
unsigned long randomMoveTime;
long int randomMovePosition;


void setup() {
  Serial.begin(115200);           // set up Serial library at 9600 bps
  
  if(!Serial) {
    delay(10);
  }

  ssInit();

  AFMS.begin();  // create with the default frequency 1.6KHz
  
  motorInit(M1);

  prevMicros = micros();

  operationMode = om_startUp;
  buttonPressed = false;
  buttonBTimePressed = millis();

  randomMoveTime = 0;
}


void loop() {
  tNow = millis();

  readJoyWing();

  if(isBPressed())
  {
    //debounce
    if(!buttonPressed && ((tNow - buttonBTimePressed) > DEBOUNCE))
    {
      buttonBTimePressed = tNow;
      
      incrementMode();

      Serial.print(operationMode);
      buttonPressed = true;
    }
  }
  else
  {
    buttonPressed = false;
  }



  //Determine desired position based on op mode
  float compFactor = 0.25;
  float compFactor2 = 0.005;
  float input;
  switch(operationMode)
  {
    case om_startUp:
      desiredPosition = 0;
    break;
    case om_manual:
    //Position
      // desiredPosition = ((1.0-compFactor)*desiredPosition) + 
      //                   (compFactor*(((float)joyX / 1024.0) * FULL_HEAD_TURN_COUNTS));

      input = ((float) joyX - 512.0);
      if(input < 50 && input > -50)
        input = 0;

      desiredPosition = ((1.0 - compFactor) * desiredPosition) + 
                        (compFactor * (desiredPosition + input));
      Serial.println(desiredPosition);
    break;
    case om_autoCycle:
      desiredPosition = FULL_HEAD_TURN_COUNTS * sin( ((float) tNow) / 1000.0);
    break;
    case om_autoRandom:

      if(tNow > randomMoveTime)
      {
        //new Random time
        randomMoveTime = tNow + (random(20, 100) * RANDOM_TIME_LIMIT / 100);

        //this random position
        randomMovePosition = random(-FULL_HEAD_TURN_COUNTS, FULL_HEAD_TURN_COUNTS);
      }

      desiredPosition = ((1.0 - compFactor2) * desiredPosition) +
                        (compFactor2 * randomMovePosition);

    break;
    case om_btManual:
      desiredPosition = 0;
    break;
    default:
      desiredPosition = 0;
    break;

  }


  //only compute pwm at a desired frequency
  unsigned long thisMicros = micros();
  if(thisMicros - prevMicros > CONTROL_LOOP_TIME_uS)
  {
    motorControl(M1, desiredPosition);
    prevMicros = thisMicros;
  }

  delayMicroseconds(750);

}
