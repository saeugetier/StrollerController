#define SERIAL_UART_INSTANCE 3
#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"
#include "encoders/smoothing/SmoothingSensor.h"
#include "voltage/GenericVoltageSense.h"
#include <PJON.h>
#include "ThroughHalfDuplexSerial.h"
#include "steering.pb.h"
#include <pb_decode.h>
#include <pb_encode.h>

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(15);
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PB13, PA9, PB14, PA10, PB15);

// encoder instance
HallSensor encoder = HallSensor(PB4, PB5, PB0, 15);
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doC(){encoder.handleC();}

SmoothingSensor smooth = SmoothingSensor(encoder, motor);

// inline current sensor instance
LowsideCurrentSense current_sense  = LowsideCurrentSense(0.002, 8, A3, A4, A5);

// commander communication instance
Commander command = Commander(Serial3);
void doMotor(char* cmd){ command.motor(&motor, cmd); }
void doTarget(char* cmd){ command.scalar(&motor.target, cmd); }

// using half duplex USART1
HardwareSerial pjonUart(PB6);

constexpr uint8_t DEVICE_ID = 44;

PJON<ThroughHalfDuplexSerial> pjon(DEVICE_ID);

GenericVoltageSense batteryVoltage(A2);
GenericVoltageSense temperatureSense(A0);

void receiver_function(uint8_t *payload, uint16_t length, const PJON_Packet_Info &packet_info) {
  pb_istream_t istream;
  SteeringMessage decoded;
  uint8_t buffer[64];

  uint32_t time = HAL_GetTick();

  digitalWrite(PD1, (time / 500) % 2);

  istream = pb_istream_from_buffer(payload, length);

  pb_decode(&istream, &SteeringMessage_msg, &decoded);

  pb_ostream_t ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  FeedbackMessage fbMessage = FeedbackMessage_init_zero;
  fbMessage.counter = decoded.counter;
  motor.target = decoded.force;

  pb_encode(&ostream, FeedbackMessage_fields, &fbMessage);
  //pjon.reply(buffer, ostream.bytes_written);
}

void error_handler(uint8_t code, uint16_t data, void *custom_pointer) 
{

}

void setup() {

  pin_SetF1AFPin(AFIO_PD01_ENABLE);

  pinMode(PD1, OUTPUT);

  pinMode(PC15, OUTPUT);

  digitalWrite(PC15, HIGH);

  digitalWrite(PD1, LOW);

  // initialize encoder sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB, doC);
  // link the motor to the sensor
  smooth.phase_correction = -_PI_6;
  motor.linkSensor(&smooth);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 36;
  driver.init();
  // link driver
  motor.linkDriver(&driver);
  // link current sense and the driver
  current_sense.linkDriver(&driver);

  motor.voltage_sensor_align = 1;
  // set control loop type to be used
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;
  motor.foc_modulation = FOCModulationType::SinePWM;

  // contoller configuration based on the controll type
  motor.PID_velocity.P = 0.5f;
  motor.PID_velocity.I = 15.0f;
  motor.PID_velocity.D = 0;
  // default voltage_power_supply
  motor.voltage_limit = 24;
  motor.current_limit = 2.5f;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;

  // angle loop controller
  motor.P_angle.P = 20;
  // angle loop velocity limit
  motor.velocity_limit = 5.0f;

  // use monitoring with serial for motor init
  // monitoring port
  Serial3.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial3);
  motor.monitor_downsample = 0; // disable intially
  //motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; // monitor target velocity and angle

  // current sense init and linking
  current_sense.init();
  motor.linkCurrentSense(&current_sense);

  // initialise motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // set the inital target value
  motor.target = 0.0f;

  // subscribe motor to the commander
  // command.add('M', doMotor, "motor");
  // command.add('T', doTarget, "target");
  command.add('M' ,doMotor, "motor");

  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  Serial3.println("Motor ready.");

  _delay(1000);

  pjonUart.begin(38400);
  pjonUart.setHalfDuplex();
  pjon.set_receiver(receiver_function);
  pjon.set_error(error_handler);
  pjon.strategy.set_serial(&pjonUart);
  pjon.begin();
}


void loop() {
  // iterative setting FOC phase voltage
  motor.loopFOC();

  // iterative function setting the outter loop target
  motor.move();

  // motor monitoring
  motor.monitor();

  // user communication
  command.run();

  // pjon update
  pjon.update();
  pjon.receive();
}