/*
AuthorS: TuMOVE TEAM NEUROREHAB PROJECT
Date: 28.07.2022 
Credits to: 
-IMU
Natalia Paredes Acuña
https://github.com/DemorianJH/ICM20948-Multi-IMU
https://github.com/ZaneL/quaternion_sensor_3d_nodejs
-SERVO AND PID
https://learn.adafruit.com/multi-tasking-the-arduino-part-1/a-clean-sweep
https://www.teachmemicro.com/arduino-pid-control-tutorial/
-FFT
https://github.com/kosme/arduinoFFT/blob/master/Examples/FFT_01/FFT_01.ino 

*/

#include <Arduino.h>
#include <Wire.h>
#include "Arduino-ICM20948-multi.h"
#include "ESP32Servo.h"
#include "arduinoFFT.h"
#include <PID_v1.h>


//Constants for the IMU setup
#define I2C_speed 400000
#define SDA 23
#define SCL 22 
#define BUFFER_SIZE 500

// Constants for printing the printVector function of the tremorDetector class
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

//Sampling frequency for the IMU and window for RMS between different sampling runs
#define SAMPLING_FREQUENCY 40
#define RMS_WINDOW 5 

//#####################################################
//IMU SETTINGS
uint8_t esp_id = 7; 
const float MAX_BATTERY_VOLTAGE = 4.2; // Max LiPoly voltage of a 3.7 battery is 4.2

ArduinoICM20948Settings icmSettings =
{
  .i2c_speed = 400000,                // i2c clock speed
  .i2c_address = 0x69,                // Usually 0x69 or 0x68
  .sda_pin = SDA,
  .scl_pin = SCL,
  .is_SPI = false,                    // Enable SPI, if disable use i2c
  .cs_pin = 10,                       // SPI chip select pin
  .spi_speed = 7000000,               // SPI clock speed in Hz, max speed is 7MHz
  .mode = 1,                          // 0 = low power mode, 1 = high performance mode
  .enable_gyroscope = true,           // Enables gyroscope output
  .enable_accelerometer = true,       // Enables accelerometer output
  .enable_magnetometer = true,        // Enables magnetometer output // Enables quaternion output
  .enable_gravity = false,             // Enables gravity vector output
  .enable_linearAcceleration = false,  // Enables linear acceleration output
  .enable_quaternion6 = false,         // Enables quaternion 6DOF output
  .enable_quaternion9 = true,         // Enables quaternion 9DOF output
  .enable_har = false,                 // Enables activity recognition
  .enable_steps = false,               // Enables step counter
  .enable_step_detector = false,      // Not really working
  .gyroscope_frequency = SAMPLING_FREQUENCY,           // Max frequency = 225, min frequency = 1
  .accelerometer_frequency = 225,       // Max frequency = 225, min frequency = 1
  .magnetometer_frequency = 70,        // Max frequency = 70, min frequency = 1 
  .gravity_frequency = 1,             // Max frequency = 225, min frequency = 1
  .linearAcceleration_frequency = 1,  // Max frequency = 225, min frequency = 1
  .quaternion6_frequency = 50,        // Max frequency = 225, min frequency = 50
  .quaternion9_frequency = 225,        // Max frequency = 225, min frequency = 50
  .har_frequency = 50,                // Max frequency = 225, min frequency = 50
  .steps_frequency = 50,              // Max frequency = 225, min frequency = 50
  .step_detector_frequency = 50
};

//###################### CLASSES AND FUNCTIONS ###############################
//IMU CLASSES
class I2C_IMU{
  private:
    
  public: 
    uint8_t id;
    ArduinoICM20948 imu;
    I2C_IMU(int i2c_id, uint8_t id):
      id(id),
      imu(i2c_id)      
    {}

// Function which reads out x, y, and z axis of the gyroscope and writes it on adressess x, y, and z from input arguments.
//Argument id is the id of the IMU.
  void run_icm20948_gyro_controller_modif(int id, float &x, float &y, float &z )
  {  
      this->id = id;
      if (imu.gyroDataIsReady())
      {
          imu.readGyroData(&x, &y, &z);
          // Checking data
          // Serial.println("x, y, z");
          // Serial.println(x);
          // Serial.println(y);
          // Serial.println(z);
      }

  }

// Function to return y axis reading from the gyroscope.
  float run_icm20948_gyro_controller_y_axis(int id)
  {  

      float xx, yy, zz;
      this->id = id;
      if (imu.gyroDataIsReady())
      {
          imu.readGyroData(&xx, &yy, &zz);
          // Checking data
          // Serial.println("x, y, z");
          // Serial.println(xx);
          // Serial.println(yy);
          // Serial.println(zz);
      }
      return yy;
  }

};

//  Battery Level Functions
void batteryLevel(){
  // A13 pin is not exposed on Huzzah32 board 
  int rawValue = analogRead(A13);

  // Reference voltage on ESP32 is 1.1V
  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html#adc-calibration
  // See also: https://bit.ly/2zFzfMT
  float voltageLevel = (rawValue / 4095.0) * 2 * 1.1 * 3.3; // calculate voltage level
  float batteryFraction = voltageLevel / MAX_BATTERY_VOLTAGE;
  
  Serial.println((String)"Raw:" + rawValue + " Voltage:" + voltageLevel + "V Percent: " + (batteryFraction * 100) + "%");
}

//Manual input function. This function maps manual input from potentiometer at A2 PIN to an energy level which will later be used
//to calculate the setpoint of the PID contoller. A small potentiometer value means the setpoint (target energy from tremor bands) 
//will be small, so damping will be turned on. High potentiometer value turns damping off.
double manualEnergy(){
  double map_var = 0;
  double potVal = analogRead(A2);
  map_var = potVal * (250.0/4100.0); // Mapping defined empirically betwe
  return map_var;
}


//#####################################################
// SERVO CLASSES

class ServoTighten
{
  Servo servo; 
  int internal_pos;           // current servo position 
  int updateInterval;         // interval between updates to the servo angle 
  int delta_angle;            // difference between desired angle and current angle
  unsigned long lastUpdate;   // last update of position 
  int increment;              //angle increment for a single update
  int safety_high = 180;      // max degree that the servo can go to 
  int safety_low = 15;        // min degree that the servo can go to (not 0 for safety reasons, to prevent overextension)

public: 
  ServoTighten(int interval, int initialpos_, int motor_increment) {
    updateInterval = interval; 
    internal_pos = initialpos_;
    increment = motor_increment; 
  }

  void Attach(int servopin){
    servo.attach(servopin);
  }

  void Detach(){
    servo.detach();
  }

  void Move(int onetimeposition){
    servo.write(onetimeposition);
  }

  //Function that updates the servo position based on the variable angle_change, which is 
  //the difference between target angle and current position computed by the PID controller.
  int Update(int angle_change){
    
    // To prevent chattering and control the speed, we only change the position if the angle change is greater or equal to the
    //increment value set in the constructor, and only when the update time is greater or equal to the update interval. 
    if((abs(angle_change) >= increment + 0) && ((millis() - lastUpdate) >= updateInterval)){  
      lastUpdate = millis();
            
      // For safety reasons, we check if the position exceeds limits. 
      if((internal_pos <= safety_high)&& (internal_pos >= safety_low)){
        if(angle_change > 0){internal_pos += increment;}
        if(angle_change < 0){internal_pos -= increment;}        
      }

    //if incrementing internal position resulted in it surpassing the safety limits,
    //we return the value within safety limits.
    if(internal_pos > safety_high) {internal_pos = safety_high;}  
    else if(internal_pos < safety_low) {internal_pos = safety_low;}
    }
    
    // If all the above criteria is fulfilled, send the move command and return
    // This part of the code is kept outside of the loop function to maintain the pulse (torque) to the arm.
    servo.write(internal_pos);
    return internal_pos;
  }

};

//#####################################################
// Root Mean Square Class
class RMS 
{
  int updateInterval = 1/SAMPLING_FREQUENCY*1000; //update interval in milliseconds
  unsigned long lastUpdate;                      // last update of position 
  int window;                                    //size of the window for the RMS
  int counter; 
  double mean, rms;
  double temp;
  double sum;
  double array_to_rms[RMS_WINDOW]; //
  

  public: 
  RMS(int window_length){
    window = window_length;
  }
  
  //Function that calculates the RMS witin the sliding window of length window.
  double calc(double sign_input){
    temp = sign_input;
    if ((millis() - lastUpdate) >= updateInterval){
      
      // Serial.println(temp);
      lastUpdate = millis();
      // shift all value left by 1 
      for(int i = window -1 ; i > 0; i--){  
        array_to_rms[i] = array_to_rms[i -1];
      }
      array_to_rms[0] = temp;

      if (counter == 1){
        mean = 0;
        rms = 0 ;
        sum = 0; 
        counter = 0;
        for(int i = 0; i < RMS_WINDOW; i++){  
          sum += pow(array_to_rms[i], 2);
        }
        mean = sum/window;
        rms = pow(mean, 0.5) ;
      }
      counter ++;
      
      }
    return rms;     
  }
};

//#####################################################
// FFT class
class tremorDetector{
        arduinoFFT FFT;   //FFT obect that computes the FFT
        uint16_t samples_ ; //number of samples
        double samplingFrequency_; //sampling frequency

        public:
        tremorDetector(uint16_t sampleNo, double samplingF) {
            FFT = arduinoFFT(); /* Create FFT object */
            samples_=sampleNo;
            samplingFrequency_=samplingF;
            }

        //Function that computes FFT and writes it into vDataReal array.
        void doFFT(double *vDataReal,  double *vDataImag){
            FFT.Windowing(vDataReal, samples_, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
            FFT.Compute(vDataReal, vDataImag, samples_, FFT_FORWARD); /* Compute FFT */
            FFT.ComplexToMagnitude(vDataReal, vDataImag, samples_); /* Compute magnitudes */
        }        
              
        //Function for printing an array, with abscissa being either samples, time or Hz. Can be used 
        //for visual inspection of raw data or the computed FFT.
        void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType){
            for (uint16_t i = 0; i < bufferSize; i++){
                double abscissa;
                /* Print abscissa value */
                    switch (scaleType)
                    {
                        case SCL_INDEX:
                            abscissa = (i * 1.0);
                        break;
                        case SCL_TIME:
                            abscissa = ((i * 1.0) / samplingFrequency_);
                        break;
                        case SCL_FREQUENCY:
                            abscissa = ((i * 1.0 * samplingFrequency_) / samples_);
                        break;
                        }

                Serial.print(abscissa, 6);
                if(scaleType==SCL_FREQUENCY)
                Serial.print("Hz");
                Serial.print(" ");
                Serial.println(vData[i], 4);
            }
            Serial.println();
        }
};

//#####################################################
// GLOBAL VARIABLES
//#####################################################

//IMU VARIABLES
I2C_IMU imu01(0,0);
char buf[BUFFER_SIZE];
float x, y, z;    //for the x, y, and z axis of the gyroscope data. Function run_icm20948_gyro_controller_modif writes directly into these variables.


//SERVO/CONTROLLER VARAIABLES
const int servopin = 26;
int delta_angle = 1;    //difference between desired and current angle
double energy = 0;
int pos = 90;     // Starting position of the servo
double angle_change = 0;     //DO not change this 
double setpoint = 35;   //setpoint for the controller, can be changed using the potentiometer input
double Kp = 5, Ki = 0, Kd=0.001;     //PID controller constants
int motor_interval = 2;  //milliseconds, update interval for servo
int motor_increment = 5;  // degrees, update anggle interval
ServoTighten servotighten1(motor_interval, pos, motor_increment);   //initializing servo tightening object with update interval, starting position, servo speed
PID controller(&energy, &angle_change, &setpoint, Kp, Ki, Kd, DIRECT); //instantiating a PID controller with input energy in the tremor frequency bands, output angle change

//RMS VARIABLES
RMS RMS_power(RMS_WINDOW); // window size
double rms_power = 0;

//FFT variables
const double samplingFrequency = SAMPLING_FREQUENCY;
const uint16_t samples = 16; //This value MUST ALWAYS be a power of 2
double vReal[samples];      //Samples are collected here. Following FFT, this variable contains FFT coefficients.
double vImag[samples];      //Will be filled with zeros since real-valued signals are analyzed.

tremorDetector detector=tremorDetector(samples, samplingFrequency); //instantiating tremorDetector class
int samplingPeriod = (1/samplingFrequency) * 1000000;

// MANUAL INPUT
const int POTENTIOMETER_INPUT_PIN = A2; // GPIO 14
int potVal = 0;

//#####################################################
//################## MAIN SETUP #######################
//#####################################################

void setup() {
  Serial.begin(115200);
  Serial.print("ID ");
  Serial.println(esp_id);
  Serial.println("Starting ICM");
  delay(1000);

  icmSettings.i2c_address = 0x69;
  icmSettings.sda_pin = SDA;
  icmSettings.scl_pin = SCL;
  Serial.println("Init imu01");
  imu01.imu.init(icmSettings, false); // only one imu

  batteryLevel();  ///// Print battery levels

  Serial.println("Attach Servo");
  servotighten1.Attach(servopin);
  servotighten1.Move(pos);
  controller.SetMode(AUTOMATIC);
  controller.SetOutputLimits(-5,5);
  int window = 100;

  pinMode(POTENTIOMETER_INPUT_PIN, INPUT);   // Manual input
   
   
  for(int count=0; count<samples; count++)
   {
    vReal[count] = 0;
    vImag[count] = 0;
   }
}


// #####################################################
// ################ MAIN LOOP  #########################
//#####################################################

void loop() {  

// FFT ANALYSIS
//collect samples in vReal array
for(int count=0; count<samples; count++){    
  imu01.imu.task0();
  x=imu01.run_icm20948_gyro_controller_y_axis(esp_id); //y axis of the gyroscope
  vReal[count] = x;
  vImag[count] = 0;
  while(micros() - microseconds < samplingPeriod){
        //empty loop to ensure the desired sampling frequency
  }
  microseconds += samplingPeriod;
}   

//FFT computation
 detector.doFFT(vReal,vImag);

//RMS over target tremor frequencies
double sum_amplitude = 0;
for (int i = 3; i < 6; i++){
    sum_amplitude += pow(vReal[i],2);
  }
sum_amplitude = sum_amplitude/3; 
sum_amplitude = pow(sum_amplitude,0.5);
Serial.print(sum_amplitude);
Serial.print("\t");

//RMS over the last |window| FFT outputs
energy = RMS_power.calc(sum_amplitude);
Serial.print(energy);
Serial.print("\t");

//Map potentiometer input to setpoint
setpoint = manualEnergy() - 1.0;    

//Copute controller output and update servo angle
controller.Compute();
pos = servotighten1.Update(angle_change);

Serial.print(setpoint);  
Serial.print("\t");
Serial.print(pos);
Serial.print("\n");
  
}