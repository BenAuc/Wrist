#ifndef __Arduino_ICM20948_MULTI_H__
#define __Arduino_ICM20948_MULTI_H__

#include <cstdio> 
#include <functional>
#include <stdint.h>

#include "Icm20948.h"

/*************************************************************************
  Defines
*************************************************************************/

typedef struct {
  int i2c_speed;
  uint8_t i2c_address;
  uint8_t sda_pin;
  uint8_t scl_pin;
  bool is_SPI;
  int cs_pin;
  int spi_speed;
  int mode;
  bool enable_gyroscope;
  bool enable_accelerometer;
  bool enable_magnetometer;
  bool enable_gravity;
  bool enable_linearAcceleration;
  bool enable_quaternion6;
  bool enable_quaternion9;
  bool enable_har;
  bool enable_steps;
  bool enable_step_detector;
  int gyroscope_frequency;
  int accelerometer_frequency;
  int magnetometer_frequency;
  int gravity_frequency;
  int linearAcceleration_frequency;
  int quaternion6_frequency;
  int quaternion9_frequency;
  int har_frequency;
  int steps_frequency;
  int step_detector_frequency;

} ArduinoICM20948Settings;

/*************************************************************************
  Class
*************************************************************************/

class ArduinoICM20948
{
  public:

    ArduinoICM20948(int i2c_id);

    //void init(TwoWire *theWire = &Wire, JTICM20948Settings settings);
    int init(ArduinoICM20948Settings settings, bool init_both);
    void i2cScan(bool init_both);
    int task0();
    int task1();
    bool gyroDataIsReady();
    bool accelDataIsReady();
    bool magDataIsReady();
    bool gravDataIsReady();
    bool linearAccelDataIsReady();
    bool quat6DataIsReady();
    bool euler6DataIsReady();
    bool quat9DataIsReady();
    bool euler9DataIsReady();
    bool harDataIsReady();
    bool stepsDataIsReady();
    bool stepTakenDataIsReady();

    void readGyroData(float *x, float *y, float *z);
    void readAccelData(float *x, float *y, float *z);
    void readMagData(float *x, float *y, float *z);
    void readGravData(float* x, float* y, float* z);
    void readLinearAccelData(float* x, float* y, float* z);
    void readQuat6Data(float *w, float *x, float *y, float *z);
    void readQuat6Data(double *w, double *x, double *y, double *z);
    void readEuler6Data(float *roll, float *pitch, float *yaw);
    void readQuat9Data(float* w, float* x, float* y, float* z);
    void readQuat9Data(double* w, double* x, double* y, double* z);
    void readEuler9Data(float* roll, float* pitch, float* yaw);
    void readHarData(char* activity);
    void readStepsData(unsigned long* steps_count);
	  void readStepTakenData();
    void readAll(float *ax, float *ay, float *az,float *gx, float *gy, float *gz);


    int getGyroBias(int * bias);
    int getAccelBias(int * bias);
    int getMagBias(int * bias);
    int setGyroBias(int * bias);
    int setAccelBias(int * bias);
    int setMagBias(int * bias);

    int spi_master_read_register(uint8_t reg, uint8_t* rbuffer, uint32_t rlen);
    int spi_master_write_register(uint8_t reg, const uint8_t* wbuffer, uint32_t wlen);
    int i2c_master_write_register(uint8_t address, uint8_t reg, uint32_t len, const uint8_t *data);
    int i2c_master_read_register(uint8_t address, uint8_t reg, uint32_t len, uint8_t *buff);
    void check_rc(int rc, const char * msg_context);
    int load_dmp3(bool init_both);
    void initiliaze_SPI(void);
    void initiliaze_I2C(int speed);
    void set_comm_interface(ArduinoICM20948Settings settings);
    inv_bool_t interface_is_SPI(void);
    int idd_io_hal_read_reg0(void *context, uint8_t reg, uint8_t *rbuffer, uint32_t rlen);
    int idd_io_hal_write_reg0(void *context, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen);
    int idd_io_hal_read_reg1(void *context, uint8_t reg, uint8_t *rbuffer, uint32_t rlen);
    int idd_io_hal_write_reg1(void *context, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen);
    void icm20948_apply_mounting_matrix(bool init_both); 
    void icm20948_set_fsr(bool init_both);
    int icm20948_sensor_setup(bool init_both);
    uint8_t icm20948_get_grv_accuracy0(void);
    uint8_t icm20948_get_grv_accuracy1(void);
    int build_sensor_event_data0(enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg);
    int build_sensor_event_data1(enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg);
    enum inv_icm20948_sensor idd_sensortype_conversion(int sensor);


    //int poll_sensor_local(struct inv_icm20948 * s, void * context);
    //int skip_sensor(struct inv_icm20948 * s, unsigned char androidSensor);
    //uint8_t inv_icm20948_updateTs(struct inv_icm20948 * s, int * data_left_in_fifo, unsigned short * total_sample_cnt, uint64_t * lastIrqTimeUs);
    //int inv_icm20948_is_streamed_sensor(uint8_t id);

    bool is_interface_SPI = false;
    int chipSelectPin;
    int com_speed;
    uint8_t I2C_Address = 0x69;
    float gyro[3];
    bool gyro_data_ready = false;
    float accel[3];
    bool accel_data_ready = false;
    float mag[3];
    bool mag_data_ready = false;
    float grav[3];
    bool grav_data_ready = false;
    float lAccel[3];
    bool linearAccel_data_ready = false;
    float quat6[4];
    bool quat6_data_ready = false;
    float euler6[3];
    bool euler6_data_ready = false;
    float quat9[4];
    bool quat9_data_ready = false;
    float euler9[3];
    bool euler9_data_ready = false;
    int har;
    bool har_data_ready = false;
    unsigned long steps;
    bool steps_data_ready = false;
    bool step_new = false;

    Icm20948 icm20948_0;
    Icm20948 icm20948_1;
    inv_icm20948_t icm_device_0;
    inv_icm20948_t icm_device_1;
    int rc = 0;
    TwoWire i2c_int;
    int i2c_id;

    float cfg_mounting_matrix[9] = {
      1.f, 0, 0,
      0, 1.f, 0,
      0, 0, 1.f
    };

    int32_t cfg_acc_fsr = 4; // Default = +/- 4g. Valid ranges: 2, 4, 8, 16
    int32_t cfg_gyr_fsr = 2000; // Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000
};


#endif