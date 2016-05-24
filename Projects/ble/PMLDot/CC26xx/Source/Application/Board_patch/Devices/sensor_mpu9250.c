/*******************************************************************************
*  Filename:       sensor_mpu9250.c
*  Revised:        $Date: 2015-02-05 10:47:02 +0100 (on, 05 feb 2015) $
*  Revision:       $Revision: 12066 $
*
*  Description:    Driver for the InvenSense MPU9250 Motion processing Unit
*
*  Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* -----------------------------------------------------------------------------
*                                          Includes
* ------------------------------------------------------------------------------
*/
#include "Board.h"
#include "sensor_mpu9250.h"
#include "sensor_opt3001.h" // For reset of I2C bus
#include "sensor.h"
#include "bsp_i2c.h"

/* -----------------------------------------------------------------------------
*                                           Constants
* ------------------------------------------------------------------------------
*/
// Sensor I2C address
#define SENSOR_I2C_ADDRESS            0x68
#define SENSOR_MAG_I2_ADDRESS         0x0C

// Registers
#define SELF_TEST_X_GYRO              0x00 // R/W
#define SELF_TEST_Y_GYRO              0x01 // R/W
#define SELF_TEST_Z_GYRO              0x02 // R/W
#define SELF_TEST_X_ACCEL             0x0D // R/W
#define SELF_TEST_Z_ACCEL             0x0E // R/W
#define SELF_TEST_Y_ACCEL             0x0F // R/W

#define XG_OFFSET_H                   0x13 // R/W
#define XG_OFFSET_L                   0x14 // R/W
#define YG_OFFSET_H                   0x15 // R/W
#define YG_OFFSET_L                   0x16 // R/W
#define ZG_OFFSET_H                   0x17 // R/W
#define ZG_OFFSET_L                   0x18 // R/W

#define SMPLRT_DIV                    0x19 // R/W
#define CONFIG                        0x1A // R/W
#define GYRO_CONFIG                   0x1B // R/W
#define ACCEL_CONFIG                  0x1C // R/W
#define ACCEL_CONFIG_2                0x1D // R/W
#define LP_ACCEL_ODR                  0x1E // R/W
#define WOM_THR                       0x1F // R/W
#define FIFO_EN                       0x23 // R/W
#define I2C_MST_CTRL     			  0x24

// .. registers 0x24 - 0x36 are not applicable to the PMLDot HW configuration

#define INT_PIN_CFG                   0x37 // R/W
#define INT_ENABLE                    0x38 // R/W
#define INT_STATUS                    0x3A // R
#define ACCEL_XOUT_H                  0x3B // R
#define ACCEL_XOUT_L                  0x3C // R
#define ACCEL_YOUT_H                  0x3D // R
#define ACCEL_YOUT_L                  0x3E // R
#define ACCEL_ZOUT_H                  0x3F // R
#define ACCEL_ZOUT_L                  0x40 // R
#define TEMP_OUT_H                    0x41 // R
#define TEMP_OUT_L                    0x42 // R
#define GYRO_XOUT_H                   0x43 // R
#define GYRO_XOUT_L                   0x44 // R
#define GYRO_YOUT_H                   0x45 // R
#define GYRO_YOUT_L                   0x46 // R
#define GYRO_ZOUT_H                   0x47 // R
#define GYRO_ZOUT_L                   0x48 // R

// .. registers 0x49 - 0x60 are not applicable to the PMLDot HW configuration
// .. registers 0x63 - 0x67 are not applicable to the PMLDot HW configuration

#define SIGNAL_PATH_RESET             0x68 // R/W
#define ACCEL_INTEL_CTRL              0x69 // R/W
#define USER_CTRL                     0x6A // R/W
#define PWR_MGMT_1                    0x6B // R/W
#define PWR_MGMT_2                    0x6C // R/W
#define FIFO_COUNT_H                  0x72 // R/W
#define FIFO_COUNT_L                  0x73 // R/W
#define FIFO_R_W                      0x74 // R/W
#define WHO_AM_I                      0x75 // R/W
#define XA_OFFSET_H      			  0x77
#define XA_OFFSET_L     			  0x78
#define YA_OFFSET_H      			  0x7A
#define YA_OFFSET_L     			  0x7B
#define ZA_OFFSET_H      			  0x7D
#define ZA_OFFSET_L      			  0x7E

// Masks is mpuConfig valiable
#define ACC_CONFIG_MASK               0x38
#define GYRO_CONFIG_MASK              0x07

// Values PWR_MGMT_1
#define MPU_SLEEP                     0x4F  // Sleep + stop all clocks
#define MPU_WAKE_UP                   0x09  // Disable temp. + intern osc

// Values PWR_MGMT_2
#define ALL_AXES                      0x3F
#define GYRO_AXES                     0x07
#define ACC_AXES                      0x38

// Data sizes
#define DATA_SIZE                     6



//Gyro Config
#define GYRO_CONFIG_FCHOICE			  0x03





// Output data rates
#define INV_LPA_0_3125HZ              0
#define INV_LPA_0_625HZ               1
#define INV_LPA_1_25HZ                2
#define INV_LPA_2_5HZ                 3
#define INV_LPA_5HZ                   4
#define INV_LPA_10HZ                  5
#define INV_LPA_20HZ                  6
#define INV_LPA_40HZ                  7
#define INV_LPA_80HZ                  8
#define INV_LPA_160HZ                 9
#define INV_LPA_320HZ                 10
#define INV_LPA_640HZ                 11
#define INV_LPA_STOPPED               255

// Bit values
#define BIT_ANY_RD_CLR                0x10
#define BIT_RAW_RDY_EN                0x01
#define BIT_WOM_EN                    0x40
#define BIT_LPA_CYCLE                 0x20
#define BIT_STBY_XA                   0x20
#define BIT_STBY_YA                   0x10
#define BIT_STBY_ZA                   0x08
#define BIT_STBY_XG                   0x04
#define BIT_STBY_YG                   0x02
#define BIT_STBY_ZG                   0x01
#define BIT_STBY_XYZA                 (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG                 (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)

// User control register
#define BIT_LATCH_EN                  0x20
#define BIT_ACTL                      0x80

// INT Pin / Bypass Enable Configuration
#define BIT_BYPASS_EN                 0x02
#define BIT_AUX_IF_EN                 0x20

// Magnetometer registers
#define MAG_WHO_AM_I                  0x00  // Should return 0x48
#define MAG_INFO                      0x01
#define MAG_ST1                       0x02  // Data ready status: bit 0
#define MAG_XOUT_L	                  0x03  // Data array
#define MAG_XOUT_H	                  0x04
#define MAG_YOUT_L	                  0x05
#define MAG_YOUT_H	                  0x06
#define MAG_ZOUT_L	                  0x07
#define MAG_ZOUT_H	                  0x08
#define MAG_ST2                       0x09  // Overflow(bit 3), read err(bit 2)
#define MAG_CNTL1                     0x0A  // Mode bits 3:0, resolution bit 4
#define MAG_CNTL2                     0x0B  // System reset, bit 0
#define MAG_ASTC                      0x0C  // Self test control
#define MAG_I2CDIS                    0x0F  // I2C disable
#define MAG_ASAX                      0x10  // x-axis sensitivity adjustment
#define MAG_ASAY                      0x11  // y-axis sensitivity adjustment
#define MAG_ASAZ                      0x12  // z-axis sensitivity adjustment

#define MAG_DEVICE_ID                 0x48

// Mode
#define MAG_MODE_OFF                  0x00
#define MAG_MODE_SINGLE               0x01
#define MAG_MODE_CONT1                0x02
#define MAG_MODE_CONT2                0x06
#define MAG_MODE_FUSE                 0x0F

// Resolution
#define MFS_14BITS                    0     // 0.6 mG per LSB
#define MFS_16BITS                    1     // 0.15 mG per LSB


#define PI 3.141592F

// Sensor selection/de-selection
#define SENSOR_SELECT()               bspI2cSelect(BSP_I2C_INTERFACE_1,SENSOR_I2C_ADDRESS)
#define SENSOR_SELECT_MAG()           bspI2cSelect(BSP_I2C_INTERFACE_1,SENSOR_MAG_I2_ADDRESS)
#define SENSOR_DESELECT()             bspI2cDeselect()

/* -----------------------------------------------------------------------------
*                           Typedefs
* ------------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
*                           Macros
* ------------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
*                           Local Functions
* ------------------------------------------------------------------------------
*/
static void sensorMpuSleep(void);
static void sensorMpu9250WakeUp(void);
static void sensorMpu9250SelectAxes(void);
static void sensorMpu9250_Callback(PIN_Handle handle, PIN_Id pinId);
static void sensorMagInit(void);
static void sensorMagEnable(bool);
static bool sensorMpu9250SetBypass(void);

/* -----------------------------------------------------------------------------
*                           Local Variables
* ------------------------------------------------------------------------------
*/
static uint8_t mpuConfig;
static uint8_t magStatus;
static uint8_t accRange;
static uint8_t accBandwidth;
static uint8_t gyroRange;
static uint8_t gyroBandwidth;
static uint8_t accRangeReg;
static uint8_t accBandwidthReg;
static uint8_t gyroRangeReg;
static uint8_t gyroBandwidthReg;
static uint8_t val;

// Magnetometer calibration
static int16_t calX;
static int16_t calY;
static int16_t calZ;

// Additional Magnetometer Calibration
static int16_t min_x = 0x7FFF;
static int16_t min_y = 0x7FFF;
static int16_t min_z = 0x7FFF;
static int16_t max_x = 0x8000;
static int16_t max_y = 0x8000;
static int16_t max_z = 0x8000;
static int16_t off_x = 0;
static int16_t off_y = 0;
static int16_t off_z = 0;
static float scale_x = 1;
static float scale_y = 1;
static float scale_z = 1;

//Gyro calibration offsets
static int16_t gyro_off_x = -170;
static int16_t gyro_off_y = 90;
static int16_t gyro_off_z = 115;

float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};

bool calOnline = 0;


// Magnetometer control
static uint8_t scale = MFS_16BITS;      // 16 bit resolution
static uint8_t mode = MAG_MODE_SINGLE;  // Operating mode

// Pins that are used by the MPU9250
static PIN_Config MpuPinTable[] =
{
    Board_MPU_INT    | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_DIS | PIN_HYSTERESIS,
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,

    PIN_TERMINATE
};
static PIN_State pinGpioState;
static PIN_Handle hMpuPin;

// The application may register a callback to handle interrupts
static MpuCallbackFn_t isrCallbackFn = NULL;

/*******************************************************************************
* @fn          sensorMpu9250PowerOn
*
* @brief       This function turns on the power supply to MPU9250
*
* @return      none
*/
void sensorMpu9250PowerOn(void)
{
  // Turn on power supply
  PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);
  delay_ms(100);
  sensorMpu9250Reset();
}

/*******************************************************************************
* @fn          sensorMpu9250PowerOff
*
* @brief       This function turns off the power supply to MPU9250
*
* @return      none
*/
void sensorMpu9250PowerOff(void)
{
  // Make sure pin interrupt is disabled
  PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_DIS);

  // Turn off power supply
  PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_OFF);

//  // Force an access on I2C bus #0 (sets the I2C lines to a defined state)
  sensorOpt3001Test();
}

/*******************************************************************************
* @fn          sensorMpu9250PowerIsOn
*
* @brief       Return 'true' if MPU power is on
*
* @return      state of MPU power
*/
bool sensorMpu9250PowerIsOn(void)
{
 return PIN_getOutputValue(Board_MPU_POWER) == Board_MPU_POWER_ON;
}

/*******************************************************************************
* @fn          sensorMpu9250RegisterCallback
*
* @brief       Register a call-back for interrupt processing
*
* @return      none
*/
void sensorMpu9250RegisterCallback(MpuCallbackFn_t pfn)
{
  isrCallbackFn = pfn;
}

/*******************************************************************************
* @fn          sensorMpu9250Init
*
* @brief       This function initializes the MPU abstraction layer.
*
* @return      True if success
*/
bool sensorMpu9250Init(void)
{
  // Pins used by MPU
  hMpuPin = PIN_open(&pinGpioState, MpuPinTable);

  // Register MPU interrupt
  PIN_registerIntCb(hMpuPin, sensorMpu9250_Callback);

  // Application callback initially NULL
  isrCallbackFn = NULL;

  return sensorMpu9250Reset();
}


/*******************************************************************************
* @fn          sensorMpu9250Reset
*
* @brief       This function resets the MPU
*
* @return      True if success
*/
bool sensorMpu9250Reset(void)
{
  bool ret;

  // Make sure pin interrupt is disabled
  PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_DIS);

  accRange = ACC_RANGE_INVALID;
  mpuConfig = 0;   // All axes off
  magStatus = 0;

  if (!SENSOR_SELECT())
  {
    return false;
  }

  // Device reset
  val = 0x80;
  sensorWriteReg(PWR_MGMT_1, &val, 1);
  SENSOR_DESELECT();

  delay_ms(100);

  ret = sensorMpu9250Test();
  if (ret)
  {
	  //TODO:Perform calibration
//	  sensorMpu9250Calibrate(gyroBias, accelBias);

    // Initial configuration
    sensorMpu9250AccSetRange(ACC_RANGE_16G);
    sensorMpu9250GyroSetRange(GYRO_RANGE_2000);
    sensorMagInit();

    // Power save
    sensorMpuSleep();
  }

  return ret;
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
//void sensorMpu9250Calibrate(float * dest1, float * dest2)
//{
//  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
//  uint16_t ii, packet_count, fifo_count;
//  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
//  uint8_t val;
//
//  if (!SENSOR_SELECT())
//  {
//    return;
//  }
//
//  // Device reset
//  val = 0x80;
//  sensorWriteReg(PWR_MGMT_1, &val, 1);
//  SENSOR_DESELECT();
//  delay_ms(100);
//
// // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
// // else use the internal oscillator, bits 2:0 = 001
//  if (!SENSOR_SELECT())
//  {
//    return;
//  }
//  val = 0x01;
//  sensorWriteReg(PWR_MGMT_1, &val, 1);
//  val = 0x00;
//  sensorWriteReg(PWR_MGMT_2, &val, 1);
//  SENSOR_DESELECT();
//  delay_ms(200);
//
//// Configure device for bias calculation
//  if (!SENSOR_SELECT())
//  {
//    return;
//  }
//  val = 0x00;
//  sensorWriteReg(INT_ENABLE, &val, 1);// Disable all interrupts
//  sensorWriteReg(FIFO_EN, &val, 1);// Disable FIFO
//  sensorWriteReg(PWR_MGMT_1, &val, 1);// Turn on internal clock source
//  sensorWriteReg(I2C_MST_CTRL, &val, 1);// Disable I2C master
//  sensorWriteReg(USER_CTRL, &val, 1);// Disable FIFO and I2C master modes
//  val = 0x0C;
//  sensorWriteReg(USER_CTRL, &val, 1);// Reset FIFO and DMP
//  SENSOR_DESELECT();
//  delay_ms(15);
//
//// Configure MPU6050 gyro and accelerometer for bias calculation
//  if (!SENSOR_SELECT())
//  {
//    return;
//  }
//  val = 0x01;
//  sensorWriteReg(CONFIG, &val, 1);// Set low-pass filter to 188 Hz
//  val = 0x00;
//  sensorWriteReg(SMPLRT_DIV, &val, 1);// Set sample rate to 1 kHz
//  sensorWriteReg(GYRO_CONFIG, &val, 1);// Set gyro full-scale to 250 degrees per second, maximum sensitivity
//  sensorWriteReg(ACCEL_CONFIG, &val, 1);// Set accelerometer full-scale to 2 g, maximum sensitivity
//
//  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
//  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g
//
//    // Configure FIFO to capture accelerometer and gyro data for bias calculation
//  val = 0x40;
//  sensorWriteReg(USER_CTRL, &val, 1);// Enable FIFO
//  val = 0x78;
//  sensorWriteReg(FIFO_EN, &val, 1);// Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
//  SENSOR_DESELECT();
//  delay_ms(40); // accumulate 40 samples in 40 milliseconds = 480 bytes
//
//// At end of sample accumulation, turn off FIFO sensor read
//  if (!SENSOR_SELECT())
//  {
//    return;
//  }
//  val = 0x00;
//  sensorWriteReg(FIFO_EN, &val, 1);// Disable gyro and accelerometer sensors for FIFO
//  sensorReadReg(FIFO_COUNT_H, &data[0], 2);// read FIFO sample count
//  fifo_count = ((uint16_t)data[0] << 8) | data[1];
//  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
//
//  for (ii = 0; ii < packet_count; ii++) {
//    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
//    sensorReadReg(FIFO_R_W, &data[0], 12);// read data for averaging
//    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
//    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
//    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
//    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
//    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
//    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
//
//    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
//    accel_bias[1] += (int32_t) accel_temp[1];
//    accel_bias[2] += (int32_t) accel_temp[2];
//    gyro_bias[0]  += (int32_t) gyro_temp[0];
//    gyro_bias[1]  += (int32_t) gyro_temp[1];
//    gyro_bias[2]  += (int32_t) gyro_temp[2];
//
//  }
//
//  SENSOR_DESELECT();
//    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
//    accel_bias[1] /= (int32_t) packet_count;
//    accel_bias[2] /= (int32_t) packet_count;
//    gyro_bias[0]  /= (int32_t) packet_count;
//    gyro_bias[1]  /= (int32_t) packet_count;
//    gyro_bias[2]  /= (int32_t) packet_count;
//
//  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
//  else {accel_bias[2] += (int32_t) accelsensitivity;}
//
//// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
//  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
//  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
//  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
//  data[3] = (-gyro_bias[1]/4)       & 0xFF;
//  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
//  data[5] = (-gyro_bias[2]/4)       & 0xFF;
//
//// Push gyro biases to hardware registers
//  if (!SENSOR_SELECT())
//  {
//    return;
//  }
//  sensorWriteReg(XG_OFFSET_H, &data[0], 1);
//  sensorWriteReg(XG_OFFSET_L, &data[1], 1);
//  sensorWriteReg(YG_OFFSET_H, &data[2], 1);
//  sensorWriteReg(YG_OFFSET_L, &data[3], 1);
//  sensorWriteReg(ZG_OFFSET_H, &data[4], 1);
//  sensorWriteReg(ZG_OFFSET_L, &data[5], 1);
//
//  SENSOR_DESELECT();
//
//// Output scaled gyro biases for display in the main program
//  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
//  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
//  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;
//
//// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
//// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
//// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
//// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
//// the accelerometer biases calculated above must be divided by 8.
//  if (!SENSOR_SELECT())
//  {
//    return;
//  }
//  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
//  sensorReadReg(XA_OFFSET_H, &data[0], 2);// Read factory accelerometer trim values
//  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
//  sensorReadReg(YA_OFFSET_H, &data[0], 2);
//  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
//  sensorReadReg(ZA_OFFSET_H, &data[0], 2);
//  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
//
//  SENSOR_DESELECT();
//
//  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
//  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
//
//  for(ii = 0; ii < 3; ii++) {
//    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
//  }
//
//  // Construct total accelerometer bias, including calculated average accelerometer bias from above
//  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
//  accel_bias_reg[1] -= (accel_bias[1]/8);
//  accel_bias_reg[2] -= (accel_bias[2]/8);
//
//  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
//  data[1] = (accel_bias_reg[0])      & 0xFF;
//  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
//  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
//  data[3] = (accel_bias_reg[1])      & 0xFF;
//  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
//  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
//  data[5] = (accel_bias_reg[2])      & 0xFF;
//  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
//
//// Apparently this is not working for the acceleration biases in the MPU-9250
//// Are we handling the temperature correction bit properly?
//// Push accelerometer biases to hardware registers
//  if (!SENSOR_SELECT())
//  {
//    return;
//  }
//  sensorWriteReg(XA_OFFSET_H, &data[0], 1);
//  sensorWriteReg(XA_OFFSET_L, &data[1], 1);
//  sensorWriteReg(YA_OFFSET_H, &data[2], 1);
//  sensorWriteReg(YA_OFFSET_L, &data[3], 1);
//  sensorWriteReg(ZA_OFFSET_H, &data[4], 1);
//  sensorWriteReg(ZA_OFFSET_L, &data[5], 1);
//  SENSOR_DESELECT();
//
//// Output scaled accelerometer biases for display in the main program
//   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
//   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
//   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
//}


/*******************************************************************************
* @fn          sensorMpu9250WomEnable
*
* @brief       Enable Wake On Motion functionality
*
* @param       threshold - wake-up trigger threshold (unit: 4 mg, max 1020mg)
*
* @return      True if success
*/
bool sensorMpu9250WomEnable(uint8_t threshold)
{
  ST_ASSERT(sensorMpu9250PowerIsOn());

  if (!SENSOR_SELECT())
  {
    return false;
  }

  // Make sure accelerometer is running
  val = 0x09;
  ST_ASSERT(sensorWriteReg(PWR_MGMT_1, &val, 1));

  // Enable accelerometer, disable gyro
  val = 0x07;
  ST_ASSERT(sensorWriteReg(PWR_MGMT_2, &val, 1));

  // Set Accel LPF setting to 184 Hz Bandwidth
  val = 0x01;
  ST_ASSERT(sensorWriteReg(ACCEL_CONFIG_2, &val, 1));

  // Enable Motion Interrupt
  val = BIT_WOM_EN;
  ST_ASSERT(sensorWriteReg(INT_ENABLE, &val, 1));

  // Enable Accel Hardware Intelligence
  val = 0xC0;
  ST_ASSERT(sensorWriteReg(ACCEL_INTEL_CTRL, &val, 1));

  // Set Motion Threshold
  val = threshold;
  ST_ASSERT(sensorWriteReg(WOM_THR, &val, 1));

  // Set Frequency of Wake-up
  val = INV_LPA_20HZ;
  ST_ASSERT(sensorWriteReg(LP_ACCEL_ODR, &val, 1));

  // Enable Cycle Mode (Accel Low Power Mode)
  val = 0x29;
  ST_ASSERT(sensorWriteReg(PWR_MGMT_1, &val, 1));

  // Select the current range
  ST_ASSERT(sensorWriteReg(ACCEL_CONFIG, &accRangeReg, 1));

  // Clear interrupt
  sensorReadReg(INT_STATUS,&val,1);

  SENSOR_DESELECT();

  mpuConfig = 0;

  // Enable pin for wake-on-motion interrupt
  PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_POSEDGE);

  return true;
}

/*******************************************************************************
* @fn          sensorMpu9250IntStatus
*
* @brief       Check whether a data or wake on motion interrupt has occurred
*
* @return      Return interrupt status
*/
uint8_t sensorMpu9250IntStatus(void)
{
  uint8_t intStatus;

  intStatus = 0;
  ST_ASSERT(sensorMpu9250PowerIsOn());

  if (SENSOR_SELECT())
  {
    if (!sensorReadReg(INT_STATUS,&intStatus,1))
    {
      intStatus = 0;
    }
    SENSOR_DESELECT();
  }

  return intStatus;
}

/*******************************************************************************
* @fn          sensorMpu9250Enable
*
* @brief       Enable accelerometer readout
*
* @param       Axes: Gyro bitmap [0..2], X = 1, Y = 2, Z = 4. 0 = gyro off
* @                  Acc  bitmap [3..5], X = 8, Y = 16, Z = 32. 0 = accelerometer off
*                    MPU  bit [6], all axes
*
* @return      None
*/
void sensorMpu9250Enable(uint16_t axes)
{
  ST_ASSERT_V(sensorMpu9250PowerIsOn());

  if (mpuConfig == 0 && axes != 0)
  {
    // Wake up the sensor if it was off
    sensorMpu9250WakeUp();
  }

  mpuConfig = axes;

  if (mpuConfig != 0)
  {
    // Enable gyro + accelerometer + magnetometer readout
    sensorMpu9250SelectAxes();
  }
  else if (mpuConfig == 0)
  {
     sensorMpuSleep();
  }
}


/*******************************************************************************
* @fn          sensorMpu9250AccSetFilter
*
* @brief       Set the DLPF of the accelerometer
*
* @param       newRange: ACCEL_CONF_2_460HZ, ACCEL_CONF_2_184HZ, ACCEL_CONF_2_92HZ, ACCEL_CONF_2_41HZ, ACCEL_CONF_2_20HZ, ACCEL_CONF_2_10HZ, ACCEL_CONF_2_5HZ
*
* @return      true if write succeeded
*/
bool sensorMpu9250AccSetFilter(uint8_t bandwidth)
{
  bool success;

  if (bandwidth == accBandwidth)
  {
    return true;
  }

  ST_ASSERT(sensorMpu9250PowerIsOn());

  if (!SENSOR_SELECT())
  {
    return false;
  }

  accBandwidthReg = bandwidth & ~(ACCEL_CONF_2_FCHOICE);

  // Apply the range
  success = sensorWriteReg(ACCEL_CONFIG_2, &accBandwidthReg, 1);
  SENSOR_DESELECT();

  if (success)
  {
    accBandwidth = bandwidth;
  }

  return success;
}


/*******************************************************************************
* @fn          sensorMpu9250AccSetRange
*
* @brief       Set the range of the accelerometer
*
* @param       newRange: ACC_RANGE_2G, ACC_RANGE_4G, ACC_RANGE_8G, ACC_RANGE_16G
*
* @return      true if write succeeded
*/
bool sensorMpu9250AccSetRange(uint8_t newRange)
{
  bool success;

  if (newRange == accRange)
  {
    return true;
  }

  ST_ASSERT(sensorMpu9250PowerIsOn());

  if (!SENSOR_SELECT())
  {
    return false;
  }

  accRangeReg = (newRange << 3);

  // Apply the range
  success = sensorWriteReg(ACCEL_CONFIG, &accRangeReg, 1);
  SENSOR_DESELECT();

  if (success)
  {
    accRange = newRange;
  }

  return success;
}

/*******************************************************************************
* @fn          sensorMpu9250AccReadRange
*
* @brief       Apply the selected accelerometer range
*
* @param       none
*
* @return      range: ACC_RANGE_2G, ACC_RANGE_4G, ACC_RANGE_8G, ACC_RANGE_16G
*/
uint8_t sensorMpu9250AccReadRange(void)
{
  ST_ASSERT(sensorMpu9250PowerIsOn());

  if (!SENSOR_SELECT())
  {
    return false;
  }

  // Apply the range
  sensorReadReg(ACCEL_CONFIG, &accRangeReg, 1);
  SENSOR_DESELECT();

  accRange = (accRangeReg>>3) & 3;

  return accRange;
}


/*******************************************************************************
* @fn          sensorMpu9250AccRead
*
* @brief       Read data from the accelerometer - X, Y, Z - 3 words
*
* @return      True if data is valid
*/
bool sensorMpu9250AccRead(uint16_t *data )
{
  bool success;

  ST_ASSERT(sensorMpu9250PowerIsOn());

  // Burst read of all accelerometer values
  if (!SENSOR_SELECT())
  {
    return false;
  }

  success = sensorReadReg(ACCEL_XOUT_H, (uint8_t*)data, DATA_SIZE);
  SENSOR_DESELECT();

  if (success)
  {
    convertToLe((uint8_t*)data,DATA_SIZE);
  }

  return success;
}

/*******************************************************************************
* @fn          sensorMpu9250GyroSetFilter
*
* @brief       Set the DLPF of the gyroscope
*
* @param       bandwidth: CONFIG_DLPF_250HZ, CONFIG_DLPF_184HZ, CONFIG_DLPF_92HZ, CONFIG_DLPF_41HZ, CONFIG_DLPF_20HZ, CONFIG_DLPF_10HZ, CONFIG_DLPF_5HZ
*
* @return      true if write succeeded
*/
bool sensorMpu9250GyroSetFilter(uint8_t bandwidth)
{
  bool success;
  uint8_t fchoice;

  if (bandwidth == gyroBandwidth)
  {
    return true;
  }

  ST_ASSERT(sensorMpu9250PowerIsOn());

  if (!SENSOR_SELECT())
  {
    return false;
  }

  gyroBandwidthReg = (bandwidth & CONFIG_DLPF_CONF);

  // Apply the range
  success = sensorWriteReg(CONFIG, &gyroBandwidthReg, 1);
  SENSOR_DESELECT();

  if (success)
  {
	  gyroBandwidth = bandwidth;
  }

  //Set Fchoice
  fchoice = 0;

  if (!SENSOR_SELECT())
  {
    return false;
  }

  success = sensorWriteReg(GYRO_CONFIG, &fchoice, 1);
  SENSOR_DESELECT();


  return success;
}

/*******************************************************************************
* @fn          sensorMpu9250GyroSetRange
*
* @brief       Set the range of the gyroscope
*
* @param       newRange: GYRO_RANGE_250, GYRO_RANGE_500, GYRO_RANGE_1000, GYRO_RANGE_2000
*
* @return      true if write succeeded
*/
bool sensorMpu9250GyroSetRange(uint8_t newRange)
{
  bool success;

  if (newRange == gyroRange)
  {
    return true;
  }

  ST_ASSERT(sensorMpu9250PowerIsOn());

  if (!SENSOR_SELECT())
  {
    return false;
  }

  gyroRangeReg = (newRange << 3);

  // Apply the range
  success = sensorWriteReg(GYRO_CONFIG, &gyroRangeReg, 1);
  SENSOR_DESELECT();

  if (success)
  {
    gyroRange = newRange;
  }

  return success;
}

/*******************************************************************************
* @fn          sensorMpu9250GyroCal
*
* @brief       Average data and set gyro offset calibration
*
* @return
*/
void sensorMpu9250GyroCalSet(int16_t *data )
{
	gyro_off_x = (gyro_off_x + data[0]) / 2;
	gyro_off_y = (gyro_off_y + data[1]) / 2;
	gyro_off_z = (gyro_off_z + data[2]) / 2;

}

/*******************************************************************************
* @fn          sensorMpu9250GyroCal
*
* @brief       Correct Gyro data for offset
*
* @return
*/
void sensorMpu9250GyroCal(int16_t *data )
{
	data[0] = data[0] - gyro_off_x;
	data[1] = data[1] - gyro_off_y;
	data[2] = data[2] - gyro_off_z;
}

/*******************************************************************************
* @fn          sensorMpu9250GyroRead
*
* @brief       Read data from the gyroscope - X, Y, Z - 3 words
*
* @return      TRUE if valid data, FALSE if not
*/
bool sensorMpu9250GyroRead(uint16_t *data )
{
  bool success;

  ST_ASSERT(sensorMpu9250PowerIsOn());

  // Select this sensor
  if (!SENSOR_SELECT())
  {
    return false;
  }

  // Burst read of all gyroscope values
  success = sensorReadReg(GYRO_XOUT_H, (uint8_t*)data, DATA_SIZE);

  SENSOR_DESELECT();

  if (success)
  {
    convertToLe((uint8_t*)data,DATA_SIZE);
  }

  if(calOnline)
  {
	  //Send data to calibration routine
	  sensorMpu9250GyroCalSet((int16_t *)data);
  }

  sensorMpu9250GyroCal((int16_t *)data);

  return success;
}


/*******************************************************************************
 * @fn          sensorMpu9250Test
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool sensorMpu9250Test(void)
{
  static bool first = true;

  ST_ASSERT(sensorMpu9250PowerIsOn());

  // Select Gyro/Accelerometer
  if (!SENSOR_SELECT())
  {
    return false;
  }

  // Make sure power is ramped up
  if (first)
  {
    delay_ms(100);
    first = false;
  }

  // Check the WHO AM I register
  ST_ASSERT(sensorReadReg(WHO_AM_I, &val, 1));
  ST_ASSERT(val == 0x71);

  SENSOR_DESELECT();

  return true;
}


/*******************************************************************************
 * @fn          sensorMpu9250AccConvert
 *
 * @brief       Convert raw data to G units
 *
 * @param       rawData - raw data from sensor
 *
 * @return      Converted value
 ******************************************************************************/
float sensorMpu9250AccConvert(int16_t *rawData)
{
  float v;

  switch (accRange)
  {
  case ACC_RANGE_2G:
    //-- calculate acceleration, unit G, range -2, +2
    v = (*rawData * 1.0) / (32768/2);
    break;

  case ACC_RANGE_4G:
    //-- calculate acceleration, unit G, range -4, +4
    v = (*rawData * 1.0) / (32768/4);
    break;

  case ACC_RANGE_8G:
    //-- calculate acceleration, unit G, range -8, +8
    v = (*rawData * 1.0) / (32768/8);
    break;

  case ACC_RANGE_16G:
    //-- calculate acceleration, unit G, range -16, +16
    v = (*rawData * 1.0) / (32768/16);
    break;
  }

  return v;
}

/*******************************************************************************
 * @fn          sensorMpu9250GyroConvert
 *
 * @brief       Convert raw data to deg/sec units
 *
 * @param       data - raw data from sensor
 *
 * @return      none
 ******************************************************************************/
float sensorMpu9250GyroConvert(int16_t *data)
{
  //-- calculate rotation, unit deg/s
	switch(gyroRange){
	case GYRO_RANGE_250:
		return (*data * 1.0) / (65536 / 500) * (PI / 180);
	case GYRO_RANGE_500:
		return (*data * 1.0) / (65536 / 1000) * (PI / 180);
	case GYRO_RANGE_1000:
		return (*data * 1.0) / (65536 / 2000)* (PI / 180);
	case GYRO_RANGE_2000:
		return (*data * 1.0) / (65536 / 4000)* (PI / 180);
	default:
		return (*data * 1.0) / (65536 / 500)* (PI / 180);
	}

}



/*******************************************************************************
* @fn          sensorMpuSleep
*
* @brief       Place the MPU in low power mode
*
* @return
*/
static void sensorMpuSleep(void)
{
  bool success;

  ST_ASSERT_V(sensorMpu9250PowerIsOn());

  if (!SENSOR_SELECT())
  {
    return;
  }

  val = ALL_AXES;
  success = sensorWriteReg(PWR_MGMT_2, &val, 1);
  if (success)
  {
    val = MPU_SLEEP;
    success = sensorWriteReg(PWR_MGMT_1, &val, 1);
  }

  SENSOR_DESELECT();
}


/*******************************************************************************
* @fn          sensorMpu9250WakeUp
*
* @brief       Exit low power mode
*
* @return      none
*/

static void sensorMpu9250WakeUp(void)
{
  bool success;

  ST_ASSERT_V(sensorMpu9250PowerIsOn());

  if (!SENSOR_SELECT())
  {
    return;
  }

  val = MPU_WAKE_UP;
  success = sensorWriteReg(PWR_MGMT_1, &val, 1);

  if (success)
  {
    // All axis initially disabled
    val = ALL_AXES;
    success = sensorWriteReg(PWR_MGMT_2, &val, 1);
    mpuConfig = 0;
  }

  if (success)
  {
    // Restore the range
    success = sensorWriteReg(ACCEL_CONFIG, &accRangeReg, 1);

    if (success)
    {
      // Clear interrupts
      success = sensorReadReg(INT_STATUS,&val,1);
    }
  }

  SENSOR_DESELECT();
}


/*******************************************************************************
* @fn          sensorMpu9250SelectAxes
*
* @brief       Select gyro, accelerometer, magnetometer
*
* @return      none
*/
static void sensorMpu9250SelectAxes(void)
{
  val = ~mpuConfig;

  if (!SENSOR_SELECT())
  {
    return;
  }

  sensorWriteReg(PWR_MGMT_2, &val, 1);

  SENSOR_DESELECT();

  sensorMagEnable(!!(mpuConfig & MPU_AX_MAG));
}


/*******************************************************************************
* @fn          sensorMpu9250SetBypass
*
* @brief       Allow the I2C bus to control the compass
*
* @return      none
*/
static bool sensorMpu9250SetBypass(void)
{
  bool success;

  if (SENSOR_SELECT())
  {
    val = BIT_BYPASS_EN | BIT_LATCH_EN;
    success = sensorWriteReg(INT_PIN_CFG, &val, 1);
    delay_ms(10);

    SENSOR_DESELECT();
  }
  else
  {
    success = false;
  }

  return success;
}


/*******************************************************************************
* @fn          sensorMagInit
*
* @brief       Initialize the compass
*
* @return      none
*/
static void sensorMagInit(void)
{
  ST_ASSERT_V(sensorMpu9250PowerIsOn());

  if (!sensorMpu9250SetBypass())
  {
    return;
  }

  if (SENSOR_SELECT_MAG())
  {
    static uint8_t rawData[3];

    // Enter Fuse ROM access mode
    val = MAG_MODE_FUSE;
    sensorWriteReg(MAG_CNTL1, &val, 1);
    delay_ms(10);

    // Get calibration data
    if (sensorReadReg(MAG_ASAX, &rawData[0], 3))
    {
      calX =  (int16_t)rawData[0] + 128;   // Return x-axis sensitivity adjustment values, etc.
      calY =  (int16_t)rawData[1] + 128;
      calZ =  (int16_t)rawData[2] + 128;
    }

    // Turn off the sensor by doing a reset
    val = 0x01;
    sensorWriteReg(MAG_CNTL2, &val, 1);

    SENSOR_DESELECT();
  }
}


/*******************************************************************************
 * @fn          sensorMpu9250MagReset
 *
 * @brief       Reset the magnetometer
 *
 * @return      none
 */
void sensorMpu9250MagReset(void)
{
  ST_ASSERT_V(sensorMpu9250PowerIsOn());

  if (sensorMpu9250SetBypass())
  {
    if (SENSOR_SELECT_MAG())
    {
      // Turn off the sensor by doing a reset
      val = 0x01;
      sensorWriteReg(MAG_CNTL2, &val, 1);
      delay_ms(10);

      // Re-enable if already active
      if (mpuConfig & MPU_AX_MAG)
      {
        val = (scale << 4) | mode;
        sensorWriteReg(MAG_CNTL1, &val, 1); // Set magnetometer data resolution and sample ODR
      }
      SENSOR_DESELECT();
    }
  }
}


/*******************************************************************************
 * @fn          sensorMagEnable
 *
 * @brief       Enable or disable the compass part of the MPU9250
 *
 * @return      none
 */
static void sensorMagEnable(bool enable)
{
  uint8_t val;

  ST_ASSERT_V(sensorMpu9250PowerIsOn());

  if (!sensorMpu9250SetBypass())
  {
    return;
  }

  if (SENSOR_SELECT_MAG())
  {
    if (enable)
    {
      // Set magnetometer data resolution and sample ODR
      val = (scale << 4) | mode;
    }
    else
    {
      // Power down magnetometer
      val = 0x00;
    }
    sensorWriteReg(MAG_CNTL1, &val, 1);

    SENSOR_DESELECT();
  }
}

/*******************************************************************************
 * @fn          sensorMpu9250MagTest
 *
 * @brief       Run a magnetometer self test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool sensorMpu9250MagTest(void)
{
  ST_ASSERT(sensorMpu9250PowerIsOn());

  // Connect magnetometer internally in MPU9250
  sensorMpu9250SetBypass();

  // Select magnetometer
  SENSOR_SELECT_MAG();

  // Check the WHO AM I register
  val = 0xFF;
  ST_ASSERT(sensorReadReg(MAG_WHO_AM_I, &val, 1));
  ST_ASSERT(val == MAG_DEVICE_ID);

  SENSOR_DESELECT();

  return true;
}

/*******************************************************************************
* @fn          sensorMpu9250MagCal
*
* @brief       Adjust magnetometer data to account hard/soft iron errors - X, Y, Z - 3 words
*
* @return      void
*/
void sensorMpu9250MagCal(int16_t *data)
{

		// Subtract the hard iron offset, and scale based on soft iron calculations
        data[0] = (data[0] - off_x) * scale_x;
        data[1] = (data[1] - off_y) * scale_y;
        data[2] = (data[2] - off_z) * scale_z;


}

/*******************************************************************************
* @fn          sensorMpu9250MagSetCal
*
* @brief       Set calibration values - X, Y, Z - 3 words
*
* @return      void
*/
void sensorMpu9250MagSetCal(int16_t *data)
{

	//Hard Iron Calibration
	//Grab max and min values
	if(data[0] > max_x)
		max_x = data[0];
	if(data[1] > max_y)
		max_y = data[1];
	if(data[2] > max_z)
		max_z = data[2];

	if(data[0] < min_x)
		min_x = data[0];
	if(data[1] < min_y)
		min_y = data[1];
	if(data[2] < min_z)
		min_z = data[2];



}

/*******************************************************************************
* @fn          sensorMpu9250MagCalcCal
*
* @brief       Calculate calibration constants given previously connected min/max data
*
* @return      void
*/
void sensorMpu9250MagCalcCal(void)
{

	//Calculate static offsets
	off_x = (min_x + max_x) / 2;
	off_y = (min_y + max_y) / 2;
	off_z = (min_z + max_z) / 2;

	//Soft Iron Calibration
	int16_t vmin_x = min_x - off_x;
	int16_t vmin_y = min_y - off_y;
	int16_t vmin_z = min_z - off_z;

	int16_t vmax_x = max_x - off_x;
	int16_t vmax_y = max_y - off_y;
	int16_t vmax_z = max_z - off_z;

	float avg_x = (vmax_x + abs(vmin_x)) / 2;
	float avg_y = (vmax_y + abs(vmin_y)) / 2;
	float avg_z = (vmax_z + abs(vmin_z)) / 2;

	float avg_rad = (avg_x + avg_y + avg_z) / 3;

	scale_x = avg_rad / avg_x;
	scale_y = avg_rad / avg_y;
	scale_z = avg_rad / avg_z;


}

/*******************************************************************************
* @fn          sensorMpu9250MagRead
*
* @brief       Read data from the compass - X, Y, Z - 3 words
*
* @return      Magnetometer status
*/
uint8_t sensorMpu9250MagRead(int16_t *data)
{

  static uint8_t calcIndex = 0;

  uint8_t val;
  uint8_t rawData[7];  // x/y/z compass register data, ST2 register stored here,
                       // must read ST2 at end of data acquisition
  magStatus = MAG_NO_POWER;
  ST_ASSERT(sensorMpu9250PowerIsOn());

  magStatus = MAG_STATUS_OK;

  // Connect magnetometer internally in MPU9250
  SENSOR_SELECT();
  val = BIT_BYPASS_EN | BIT_LATCH_EN;
  if (!sensorWriteReg(INT_PIN_CFG, &val, 1))
  {
    magStatus = MAG_BYPASS_FAIL;
  }
  SENSOR_DESELECT();

  if (magStatus != MAG_STATUS_OK)
  {
    return false;
  }

  // Select this sensor
  SENSOR_SELECT_MAG();

  if (sensorReadReg(MAG_ST1,&val,1))
  {
    // Check magnetometer data ready bit
    if (val & 0x01)
    {
      // Burst read of all compass values + ST2 register
      if (sensorReadReg(MAG_XOUT_L, &rawData[0],7))
      {
        val = rawData[6]; // ST2 register

        // Check if magnetic sensor overflow set, if not then report data
        if(!(val & 0x08))
        {

          // Data from magnetometer isn't aligned to other sensors so swizzle axis to correct.
//Aligned to MPU axis
//          data[1] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value - Swap X and Y
//          data[0] = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian - Swap X and Y
//          data[2] = (-1) * (((int16_t)rawData[5] << 8) | rawData[4]);  // Invert Z to align


//aligned to paraglider app axis
		  data[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value - Swap X and Y
		  data[1] = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian - Swap X and Y
		  data[2] = ((int16_t)rawData[5] << 8) | rawData[4];

          // Sensitivity adjustment
          data[1] = data[1] * calX >> 8;
          data[0] = data[0] * calY >> 8;
          data[2] = data[2] * calZ >> 8;

          sensorMpu9250MagSetCal(data);

          sensorMpu9250MagCal(data);

          if(calcIndex == 0xFF){
        	  sensorMpu9250MagCalcCal();
          }
          calcIndex++;


        }
        else
        {
          magStatus = MAG_OVERFLOW;
        }
      }
      else
      {
        magStatus = MAG_READ_DATA_ERR;
      }
    }
    else
    {
      magStatus = MAG_DATA_NOT_RDY;
    }
  }
  else
  {
    magStatus = MAG_READ_ST_ERR;
  }

  // Start new conversion
  val = (scale << 4) | mode;
  sensorWriteReg(MAG_CNTL1, &val, 1); // Set magnetometer data resolution and sample ODR

  SENSOR_DESELECT();

  return magStatus;
}


/*******************************************************************************
* @fn          sensorMpu9250MagStatus
*
* @brief       Return the magnetometer status
*
* @return      mag status
*/
uint8_t sensorMpu9250MagStatus(void)
{
  return magStatus;
}

/*******************************************************************************
 *  @fn         sensorMpu9250_Callback
 *
 *  Interrupt service routine for the MPU
 *
 *  @param      handle PIN_Handle connected to the callback
 *
 *  @param      pinId  PIN_Id of the DIO triggering the callback
 *
 *  @return     none
 ******************************************************************************/
static void sensorMpu9250_Callback(PIN_Handle handle, PIN_Id pinId)
{
  if (pinId == Board_MPU_INT)
  {
    if (isrCallbackFn != NULL)
    {
      isrCallbackFn();
    }
  }
}
