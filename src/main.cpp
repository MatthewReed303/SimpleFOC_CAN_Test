#include <Arduino.h>
#include <SimpleFOCCAN.h>
/*
//TEST ENCODER IS WORKING CORRECTLY

#include <SimpleFOC.h>

// encoder instance
Encoder encoder = Encoder(PB4, PB5, 1800, PC9);

// Interrupt routine intialisation
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doI(){encoder.handleIndex();}

void setup() {
  // monitoring port
  Serial.begin(115200);
  
  // initialise encoder hardware
  encoder.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB, doI);

  Serial.println("Encoder ready");
  _delay(1000);
}

void loop() {
  // display the angle and the angular velocity to the terminal
  Serial.print(encoder.getAngle());
  Serial.print("\t");
  Serial.println(encoder.getVelocity());
}

*/

/*

// TEST MOTOR IN OPEN LOOP MODE
#include <SimpleFOC.h>

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number, phase resistance (optional) );
BLDCMotor motor = BLDCMotor(7);
// BLDCDriver6PWM(IN1_H, IN1_L, IN2_H, IN2_L, IN3_H, IN3_L, enable(optional))
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PB13, PA9, PB14, PA10, PB15, PB12);

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }

void setup() {

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 24;
  driver.pwm_frequency = 50000;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // limiting motor movements
  motor.phase_resistance = 0.039; // [Ohm]
  motor.current_limit = 15;   // [Amps] - if phase resistance defined
  motor.voltage_limit = 24;   // [V] - if phase resistance not defined
  motor.velocity_limit = 150; // [rad/s] cca 50rpm

  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  motor.init();

  // add target command T
  command.add('T', doTarget, "target velocity");

  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}
void loop() {

  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  motor.move();

  // user communication
  command.run();
}


*/


#include <SimpleFOC.h>

// Motor instance
BLDCMotor motor = BLDCMotor(7);
// BLDCDriver3PWM(IN1, IN2, IN3, enable(optional))
//BLDCDriver3PWM driver = BLDCDriver3PWM(PB6, PB7, PB8, PB5);
// BLDCDriver6PWM(IN1_H, IN1_L, IN2_H, IN2_L, IN3_H, IN3_L, enable(optional))
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PB13, PA9, PB14, PA10, PB15, PB12);

// encoder instance
Encoder encoder = Encoder(PB4, PB5, 256, PC9);

// Interrupt routine intialisation
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doI(){encoder.handleIndex();}

//CAN Bus Communication Instance
CANDriver canD = CANDriver(PB8, PB9);
CANCommander canCommand = CANCommander(canD);

Commander command = Commander(Serial);

void doCommander(char* cmd) { command.motor(&motor, cmd); }
void doCommanderCAN(char* cmd) { canCommand.motor(&motor, cmd); }
  



// commander communication instance
//Commander command = Commander(Serial);
//void doMotor(char* cmd){ command.motor(&motor, cmd); }

// current sensor
//InlineCurrentSense current_sense = InlineCurrentSense(0.0005, 10, PC0, PC1);


void setup() {

  // monitoring port
  Serial.begin(115200);

  // default voltage_power_supply
  motor.voltage_limit = 12;
  motor.velocity_limit = 500;
  motor.current_limit = 100;
  motor.phase_resistance = 0.12; // [Ohm]
  motor.motion_downsample = 1;
  motor.monitor_downsample = 100;

  // aligning voltage [V]
  motor.voltage_sensor_align = 1.5;
  // index search velocity [rad/s]
  motor.velocity_index_search = 1.0;

  // initialize encoder sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB, doI); 
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // set control loop type to be used
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity;

  // controller configuration based on the control type 
  // velocity loop PID
  motor.PID_velocity.P = 0.8;
  motor.PID_velocity.I = 10;
  motor.PID_velocity.D = 0.005;
  motor.PID_velocity.output_ramp = 1000.0;
  motor.PID_velocity.limit = 500;
  // Low pass filtering time constant 
  motor.LPF_velocity.Tf = 0.05;
  // angle loop PID
  motor.P_angle.P = 50.0;
  motor.P_angle.I = 0.0;
  motor.P_angle.D = 10.5;
  motor.P_angle.output_ramp = 1000.0;
  motor.P_angle.limit = 500.0;
  // Low pass filtering time constant 
  motor.LPF_angle.Tf = 0.0;
  // current q loop PID 
  motor.PID_current_q.P = 0.5;
  motor.PID_current_q.I = 0.1;
  motor.PID_current_q.D = 0.0;
  motor.PID_current_q.output_ramp = 50;
  motor.PID_current_q.limit = 1;
  // Low pass filtering time constant 
  motor.LPF_current_q.Tf = 0.1;
  // current d loop PID
  motor.PID_current_d.P = 1;
  motor.PID_current_d.I = 0.5;
  motor.PID_current_d.D = 0.01;
  motor.PID_current_d.output_ramp = 100;
  motor.PID_current_d.limit = 2;
  // Low pass filtering time constant 
  motor.LPF_current_d.Tf = 0.00;
  
  
  // velocity low pass filtering time constant
  //motor.LPF_velocity.Tf = 0.01;

  // use monitoring with serial for motor init

  
  //CAN Setup
  command.add('M', doCommander, (char*)"motor");
	canCommand.add('M', doCommanderCAN, (char*)"motor");
  // comment out if not needed
  motor.useMonitoring(Serial);
  
  //motor.monitor_downsample = 0; // initially disable the real-time monitor

  // current sense init and linking
  //current_sense.init();
  //current_sense.skip_align = true;
  //current_sense.gain_a *= -1;
  //current_sense.gain_b *= -1;
  //motor.linkCurrentSense(&current_sense);

  // initialise motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC(); 

  // set the inital target value
  motor.target = 0;

  // subscribe motor to the commander
  //command.add('M', doMotor, "motor");

  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  Serial.println(F("Motor commands sketch | Initial motion control > torque/current : target 0Amps."));
  
  _delay(1000);
}


void loop() {
  // iterative setting FOC phase voltage
  motor.loopFOC();

  // iterative function setting the outer loop target
  motor.move();

  // motor monitoring
  //motor.monitor();

  // CAN Bus Communication
  canCommand.runWithCAN();
	
  // user communication
  command.run();
  
}
