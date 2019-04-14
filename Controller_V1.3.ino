


#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

//Creates sensor object
LSM9DS1 imu;

// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

//Sets the speed the looped code is executed
#define SAMPLE_TIME 10

//Setup a serial connection on pins 4,5
SoftwareSerial bt(4,5);

// Required variables

// state variable
int state = 0;

//used as counter
int i;

//used for timing
unsigned long last, last2;

//angle variables
float ang = 0;
float ang_A;
float ang_G;
float avgz, avgy;
float accelz, accely;

// IMU Gyro bias values
double bias1 = 0;
double biasp1 = 0;
double bias2 = 0;

//Controller Variables
float Integral[100];
float integral = 0;
float setpoint = 0;
float turn_setpoint = 0;
float heading = 0;

void setup() {

  //begin serial comms
  bt.begin(9600);


  //Creates an array 'Integral' of 100 zeros for use in the Integral Controller.
  for (int i = 0; i < 100; i++) {
        Integral[i] = 0;        
        }
    
  //establish motor direction toggle pins
  pinMode(12, OUTPUT); //CH A -- HIGH = forwards and LOW = backwards
  pinMode(13, OUTPUT); //CH B -- HIGH = forwards and LOW = backwards?
  
  //establish motor brake pins
  pinMode(9, OUTPUT); //brake (disable) CH A
  pinMode(8, OUTPUT); //brake (disable) CH B
  

  Serial.begin(115200);
  
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }

  
    // Initial angle calibration
    imu.readAccel();
    avgz = imu.calcAccel(imu.az);
    avgy = imu.calcAccel(imu.ay);
    last = 0;
    i = 2;

    //average the accelerometers for 100 samples for initial angle
    //also read in 100 values of gyro to calculate bias at rest
    while (i < 102)
    {
      if ( imu.accelAvailable() && (millis() - last) > SAMPLE_TIME)
      { imu.readAccel();
        imu.readGyro();
        accelz = imu.calcAccel(imu.az);
        accely = imu.calcAccel(imu.ay);
        avgz = (avgz * (i - 1) + accelz) / i;
        avgy = (avgy * (i - 1) + accely) / i;

        // Gyro bias calibration
        biasp1 += -1.35 * imu.calcGyro(imu.gx);
        

        i++;
        last = millis();
        float test = atan(avgy / avgz);
      }
    }
    bias1 = biasp1 / 100;
   
    biasp1=0;
    
    // Initial angle from calibration
    ang = atan(avgy / avgz);
}



  
  
void loop() {

  
  
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }

  
        //Angle estimation and control
  if ((millis() - last2) > SAMPLE_TIME)
  {

      ang_G = ang + ((-1.35 * (imu.calcGyro(imu.gx)) - bias1) * 0.01);
      ang_A = 57.2957795 * atan2(imu.calcAccel(imu.ay), imu.calcAccel(imu.az));
      ang = (ang_G * 0.98) + (ang_A * 0.02);

      int i_time = 10; //number of 10ms
      //PID values scaled by 1/0.035 to take into account 0-255 PWM to 0-9V conversion
      float P = 15;
      float I = 2;
      float D = 1;
      int setpoint_scale = 1;
      int turn_scale = 5;

      //Bluetooth Control
      if (bt.available()) {
            char bt_data = bt.read();
            Serial.println(bt_data);
            if (bt_data == 'F') {
              setpoint = setpoint + setpoint_scale;
              
              }
            if (bt_data == 'B' ) {
              setpoint = setpoint - setpoint_scale;
              
              }
            if (bt_data == 'r') {
              setpoint = 0;
              turn_setpoint = 0;
     
             }
            if (bt_data == 'L') {
              turn_setpoint = turn_setpoint + turn_scale;
     
             }
            if (bt_data == 'R') {
              turn_setpoint = turn_setpoint - turn_scale;
     
             }
            
      }


      
      //Exponential Decay of motion signals
      setpoint = setpoint * 0.990832;
      turn_setpoint = turn_setpoint * 0.990832;
     
     // Serial.println(setpoint);
     
      float ang_corrected = ang - 1.3; // angle bias -- larger angle tilts away from port 
      float error = ang_corrected + setpoint;
      float w = (-1.35 * (imu.calcGyro(imu.gx)) - bias1);
      float newIntegral[i_time];
      newIntegral[0] = error;
      for (int i = 0; i < i_time - 1; i++) {
        newIntegral[i+1] = Integral[i];        
        }
      float* Integral = newIntegral;
      integral = integral + error - Integral[i_time-1]; 

      //Generates control signal for motors
      float motor = ((error) * P)+(w*D)+(integral*I);
      //Serial.println(ang_corrected);
      //Serial.println(motor);
      //Safety feature -- if angle > 45 degrees, motors are disabled until reset
      if (abs(ang) > 45) {
        motor_control(0, 0);
        while (true)
        {
          }
        }

      //motor control function  
      motor_control(motor, turn_setpoint);
      //Serial.println((ang_corrected) * P);
     //Serial.println(integral*I);
     // Serial.println(w*D);
     // Serial.println(ang_corrected);
     
      last2 = millis();
  }
}


//take in variable 'set' which varies the voltage to the motors, and variable 'turn' which puts a voltage differential on the motors to control orintation.
void motor_control(float set, float turn) {
  
float set_1 = set - turn;
float set_2 = set + turn;
  
  int forward_1 = 0;
  int forward_2 = 0;
  if (set_1 > 0) {
    forward_1 = 1;
  }
  if (set_2 > 0) {
    forward_2 = 1;
  }
  set_1 = abs(set_1);
  if (set_1 > 255) {
    set_1 = 255;
  } 
  int speed_val_1 = set_1;

  set_2 = abs(set_2);
  if (set_2 > 255) {
    set_2 = 255;
  } 
  
  int speed_val_2 = set_2;
  digitalWrite(9, LOW);  //ENABLE CH A
  digitalWrite(8, LOW);  //ENABLE CH B

  digitalWrite(12, forward_1);   //Sets direction of CH A
  digitalWrite(13, !forward_2);   //Sets direction of CH B

  analogWrite(3, speed_val_1);   //Moves CH A
  analogWrite(11, speed_val_2);   //Moves CH B
  
  }
