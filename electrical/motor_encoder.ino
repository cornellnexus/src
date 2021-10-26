/*
 * This is code for a test system to characterize a motor's response running in a
 * basic control loop. This implementation uses Arduino, but the actual system is
 * built using a Raspberry Pi.
*/

#include <PID_v1.h>

#define IN1 4
#define IN2 5
#define INT_PIN 2
#define ENA 7
#define PPR 500
#define GR 38.3 // Gear ratio

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;

volatile long pulse_counter = 0;
volatile int edge_counter = 0;

const double angDis = PI / (PPR * GR);

// Time to run the control loop
int run_time = 20;
unsigned long time_elapsed = 0;

int motor_speed;

//MOTOR DRIVER: Definitions Arduino pins connected to input H Bridge

void setup() {
  Serial.begin(115200);
  // create interrupt on rising edge of encoder
  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), IRQ_Handler, RISING);

  //MOTOR DRIVER: Set the output pins

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void loop() {
  time_elapsed = millis();
  
  // Stop updating everything if we run for more than 20sec
  if (time_elapsed >= 20000)
    while(1);  
  
  Serial.println(pulse_counter * angDis);

  // Put control loop here
}

void motor_forward( void ) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Send PWM output from CTRL loop
  analogWrite(ENA, motor_speed);
}

// Interrupt Handler
void IRQ_Handler()
{
  if (edge_counter == 0)
  {
    edge_counter++;
  }
  else
  {
    pulse_counter++;
    edge_counter = 0;
  }
  if (pulse_counter * angDis >= 2 * PI)
    pulse_counter = 0;
}

/*
Possible future implementation --> use both waveforms to increase resolution of encoder
*/
