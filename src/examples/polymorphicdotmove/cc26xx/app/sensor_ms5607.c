/*******************************************************************************
*  Filename:       sensor_bmp280.c
*  Revised:        $Date: 2014-02-05 10:47:02 +0100 (on, 05 feb 2014) $
*  Revision:       $Revision: 12066 $
*
*  Description:    Driver for the Bosch BMP280 Pressure Sensor
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


#include <ti/sysbios/knl/Task.h>

#include "board.h"
#include "sensor_ms5607.h"
#include "SensorUtil.h"
#include "SensorI2C.h"
#include "stddef.h"
#include "math.h"

/* -----------------------------------------------------------------------------
*                                           Constants and macros
* ------------------------------------------------------------------------------
*/


// I2C commands of chip
#define MS5xxx_CMD_RESET	0x1E    // perform reset
#define MS5xxx_CMD_ADC_READ 0x00    // initiate read sequence
#define MS5xxx_CMD_ADC_CONV 0x40    // start conversion
#define MS5xxx_CMD_ADC_D1   0x00    // read ADC 1
#define MS5xxx_CMD_ADC_D2   0x10    // read ADC 2
#define MS5xxx_CMD_ADC_256  0x00    // set ADC oversampling ratio to 256
#define MS5xxx_CMD_ADC_512  0x02    // set ADC oversampling ratio to 512
#define MS5xxx_CMD_ADC_1024 0x04    // set ADC oversampling ratio to 1024
#define MS5xxx_CMD_ADC_2048 0x06    // set ADC oversampling ratio to 2048
#define MS5xxx_CMD_ADC_4096 0x08    // set ADC oversampling ratio to 4096
#define MS5xxx_CMD_PROM_RD  0xA0    // initiate readout of PROM registers


// Sensor selection/deselection
#define SENSOR_SELECT()     SensorI2C_select(SENSOR_I2C_1,Board_MS5607_ADDR)
#define SENSOR_DESELECT()   SensorI2C_deselect()

/* -----------------------------------------------------------------------------
*                                           Type Definitions
* ------------------------------------------------------------------------------
*/


/* -----------------------------------------------------------------------------
*                                           Local Functions
* ------------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
*                                           Local Variables
* ------------------------------------------------------------------------------
*/
uint16_t C[8];
double P;
double TEMP;


/*******************************************************************************
 * @fn          sensorMs5607Init
 *
 * @brief       Initialize the sensor
 *
 * @return      true if success
 */
bool sensorMs5607Init(void)
{
//	bool ret;
	uint8_t val[2];
	uint8_t i;

	if (!SENSOR_SELECT())
		return false;

	SensorI2C_writeReg(MS5xxx_CMD_RESET, 0, 0);
	SENSOR_DESELECT();

//	delay(3);
	DELAY_MS(3);

	if (!SENSOR_SELECT())
		return false;

	for(i = 0; i < 8; i++)
	{
		C[i]=0x0000;
		SensorI2C_writeReg(MS5xxx_CMD_PROM_RD+2*i, val, sizeof(val));
		C[i] = (val[0] << 8) + val[1];

	}

	SENSOR_DESELECT();


	return true;
}


/*******************************************************************************
 * @fn          sensorMs5607Enable
 *
 * @brief       Enable/disable measurements
 *
 * @param       enable - flag to turn the sensor on/off
 *
 * @return      none
 */
void sensorMs5607Enable(bool enable)
{
//  uint8_t val;

//  if (enable)
//  {
//    // Enable forced mode; pressure oversampling 4; temp. oversampling 1
//    val = PM_NORMAL | OSRSP(4) | OSRST(1);
//  }
//  else
//  {
//    val = PM_OFF;
//  }
//
//  if (!SENSOR_SELECT())
//    return;
//
//  sensorWriteReg(ADDR_CTRL_MEAS, &val, sizeof(val));
//  SENSOR_DESELECT();
}


/*******************************************************************************
 * @fn          sensorMs5607Read
 *
 * @brief       Read temperature and pressure data
 *
 * @param       data - buffer for temperature and pressure (8 bytes)
 *
 * @return      TRUE if valid data
 */
bool sensorMs5607Read(uint8_t *data)
{
//	bool success;
//	uint64_t D1=0, D2=0;
//
//	double dT;
//	double OFF;
//	double SENS;
//
//	volatile float temp_f, pres_f;

  //TODO: READ OUT getTemp, getPres

//  if (!SENSOR_SELECT())
//    return false;
//
//  success = sensorReadReg( ADDR_PRESS_MSB, data, BMP_DATA_SIZE);
//  SENSOR_DESELECT();
//
//  if (success)
//  {
//    // Validate data
//    success = !(data[0]==0x80 && data[1]==0x00 && data[2]==0x00);
//  }
//
//  if (!success)
//  {
//    sensorSetErrorData(data,BMP_DATA_SIZE);
//  }


//	if (!SENSOR_SELECT())
//		return false;
//
//	//TODO: Hack to fix barometer read problem
//	D1=sensorMs5607ReadAdc(MS5xxx_CMD_ADC_D1+MS5xxx_CMD_ADC_256);
//	D2=sensorMs5607ReadAdc(MS5xxx_CMD_ADC_D2+MS5xxx_CMD_ADC_2048);
//	D1=sensorMs5607ReadAdc(MS5xxx_CMD_ADC_D1+MS5xxx_CMD_ADC_2048);
//
//	SENSOR_DESELECT();


	unsigned long D1=0, D2=0;

	double dT;
	double OFF;
	double SENS;

	if (!SENSOR_SELECT())
		return false;

	//TODO: Hack to fix barometer read problem
//	D1=sensorMs5607ReadAdc(MS5xxx_CMD_ADC_D1+MS5xxx_CMD_ADC_256);
	D2=sensorMs5607ReadAdc(MS5xxx_CMD_ADC_D2+MS5xxx_CMD_ADC_2048);
	D1=sensorMs5607ReadAdc(MS5xxx_CMD_ADC_D1+MS5xxx_CMD_ADC_2048);

	SENSOR_DESELECT();

	// calculate 1st order pressure and temperature (MS5607 1st order algorithm)
	dT=D2-C[5]*pow(2,8);
	OFF=C[2]*pow(2,17)+dT*C[4]/pow(2,6);
	SENS=C[1]*pow(2,16)+dT*C[3]/pow(2,7);
	TEMP=(2000+(dT*C[6])/pow(2,23));
	P=(((D1*SENS)/pow(2,21)-OFF)/pow(2,15));

	// perform higher order corrections
	double T2=0., OFF2=0., SENS2=0.;
	if(TEMP<2000) {
	  T2=dT*dT/pow(2,31);
	  OFF2=61*(TEMP-2000)*(TEMP-2000)/pow(2,4);
	  SENS2=2*(TEMP-2000)*(TEMP-2000);
	  if(TEMP<-1500) {
	    OFF2+=15*(TEMP+1500)*(TEMP+1500);
	    SENS2+=8*(TEMP+1500)*(TEMP+1500);
	  }
	}

	TEMP-=T2;
	OFF-=OFF2;
	SENS-=SENS2;
	P=(((D1*SENS)/pow(2,21)-OFF)/pow(2,15));

	*((float *)data) = (float) P;
	*((float *)(data+4)) = (float) TEMP;
//	pres_f = (float) P;
//	temp_f = (float) TEMP;

  return true;
}


/*******************************************************************************
 * @fn          sensorMs5607Convert
 *
 * @brief       Convert raw data to object and ambience temperature
 *
 * @param       data - raw data from sensor
 *
 * @param       temp - converted temperature
 *
 * @param       press - converted pressure
 *
 * @return      none
 ******************************************************************************/
void sensorMs5607Convert(uint8_t *data, int32_t *temp, uint32_t *press)
{
//  int32_t utemp, upress;
//  Ms5607Calibration_t *p = (Ms5607Calibration_t *)calData;
//	int32_t v_x1_u32r;
//	int32_t v_x2_u32r;
//  int32_t t_fine;
//	uint32_t pressure;

//  // Pressure
//  upress = (int32_t)((((uint32_t)(data[0])) << 12) |
//                     (((uint32_t)(data[1])) << 4) | ((uint32_t)data[2] >> 4));
//
//  // Temperature
//  utemp = (int32_t)((( (uint32_t) (data[3])) << 12) |
//                    (((uint32_t)(data[4])) << 4) | ((uint32_t)data[5] >> 4));
//
//  // Compensate temperature
//	v_x1_u32r  = ((((utemp >> 3) - ((int32_t)p->dig_T1 << 1))) *
//                ((int32_t)p->dig_T2)) >> 11;
//	v_x2_u32r  = (((((utemp >> 4) - ((int32_t)p->dig_T1)) *
//                  ((utemp >> 4) - ((int32_t)p->dig_T1))) >> 12) *
//                ((int32_t)p->dig_T3)) >> 14;
//	t_fine = v_x1_u32r + v_x2_u32r;
//  *temp = (int32_t)((t_fine * 5 + 128) >> 8);
//
//  // Compensate pressure
//	v_x1_u32r = (((int32_t)t_fine) >> 1) - (int32_t)64000;
//	v_x2_u32r = (((v_x1_u32r >> 2) * (v_x1_u32r >> 2)) >> 11) *
//    ((int32_t)p->dig_P6);
//	v_x2_u32r = v_x2_u32r + ((v_x1_u32r * ((int32_t)p->dig_P5)) << 1);
//	v_x2_u32r = (v_x2_u32r >> 2) + (((int32_t)p->dig_P4) << 16);
//	v_x1_u32r = (((p->dig_P3 * (((v_x1_u32r >> 2) *
//                 (v_x1_u32r >> 2)) >> 13)) >> 3) +
//                  ((((int32_t)p->dig_P2) * v_x1_u32r) >> 1)) >> 18;
//	v_x1_u32r = ((((32768+v_x1_u32r)) * ((int32_t)p->dig_P1))	>> 15);
//
//	if (v_x1_u32r == 0)
//		return; /* Avoid exception caused by division by zero */
//
//	pressure = (((uint32_t)(((int32_t)1048576) - upress) -
//               (v_x2_u32r >> 12))) * 3125;
//	if (pressure < 0x80000000)
//		pressure = (pressure << 1) / ((uint32_t)v_x1_u32r);
//	else
//		pressure = (pressure / (uint32_t)v_x1_u32r) * 2;
//	v_x1_u32r = (((int32_t)p->dig_P9) *
//               ((int32_t)(((pressure >> 3) * (pressure >> 3)) >> 13))) >> 12;
//	v_x2_u32r = (((int32_t)(pressure >> 2)) * ((int32_t)p->dig_P8)) >> 13;
//	pressure = (uint32_t)((int32_t)pressure +
//                        ((v_x1_u32r + v_x2_u32r + p->dig_P7) >> 4));
//
//  *press = pressure;
}

/*******************************************************************************
 * @fn          sensorMs5607Test
 *
 * @brief       Run a sensor self-test
 *
 * @return      true if passed
 */
bool sensorMs5607Test(void)
{
//  uint8_t val;

  // Select this sensor on the I2C bus
//  if (!SENSOR_SELECT())
//    return false;
//
//  // Check reset values
//  ST_ASSERT(sensorReadReg(ADDR_PROD_ID, &val, sizeof(val)));
//  ST_ASSERT(val == VAL_PROD_ID);
//
//  ST_ASSERT(sensorReadReg(ADDR_CONFIG, &val, sizeof(val)));
//  ST_ASSERT(val == VAL_CONFIG);
//
//  // Check that registers can be written
//  val = VAL_CTRL_MEAS_TEST;
//  ST_ASSERT(sensorWriteReg(ADDR_CTRL_MEAS, &val, sizeof(val)));
//  ST_ASSERT(sensorReadReg(ADDR_CTRL_MEAS, &val, sizeof(val)));
//  ST_ASSERT(val == VAL_CTRL_MEAS_TEST);
//
//  // Reset the sensor
//  val = VAL_RESET_EXECUTE;
//  ST_ASSERT(sensorWriteReg(ADDR_RESET, &val, sizeof(val)));
//
//  // Check that CTRL_MEAS register has reset value
//  ST_ASSERT(sensorReadReg(ADDR_CTRL_MEAS, &val, sizeof(val)));
//  ST_ASSERT(val == VAL_CTRL_MEAS);
//
//  SENSOR_DESELECT();

  return true;
}

//void ReadProm() {
//	send_cmd(MS5xxx_CMD_RESET);
//	delay(3);
//
//	for(uint8_t i=0;i<8;i++)
//	{
//	    C[i]=0x0000;
//	    send_cmd(MS5xxx_CMD_PROM_RD+2*i);
//	    _Wire->requestFrom(i2caddr, 2);
//
//	    unsigned int c = _Wire->read();
//	    C[i] = (c << 8);
//	    c = _Wire->read();
//	    C[i] += c;
//	    _Wire->endTransmission(true);
//	}
//
//}
//
//unsigned int Calc_CRC4(unsigned char poly)
//{
//    int cnt;                   		// simple counter
//    unsigned int n_rem;                 // CRC remainder
//    unsigned int crc_read;              // original value of the CRC
//    unsigned int l_pol = poly;
//    unsigned char n_bit;
//
//    l_pol = ( l_pol << 8 ) & 0xf000;	// shift bits and apply mask
//    n_rem = 0x0000;
//
//    crc_read = C[ 7 ];                  // save read RCR
//    C[ 7 ] = ( 0xFF00 & ( C[ 7 ] ) );   // CRC byte is replaced by 0
//    for ( cnt = 0; cnt < 16; cnt++ )    // operation is performed on bytes
//    {// choose LSB or MSB
//        if ( cnt % 2 == 1 ) n_rem ^= ( unsigned short ) ( ( C[ cnt >> 1 ] ) & 0x00FF );
//        else n_rem ^= ( unsigned short ) ( ( C[ cnt >> 1 ] >> 8) & 0x00FF );
//
//        for ( n_bit = 8; n_bit > 0; n_bit-- )
//        {
//            if ( n_rem & ( 0x8000 ) )
//            {
//            	n_rem = ( n_rem << 1 ) ^ l_pol;
//            }
//            else
//            {
//                n_rem = ( n_rem << 1 );
//            }
//        }
//    }
//    C[ 7 ] = crc_read;
//    n_rem = (0x000F & (n_rem >> 12)); // final 4-bit remainder is CRC code
//    return n_rem;
//}
//
//unsigned int Read_CRC4()
//{
//
//    unsigned int crc_read = ( 0x000F & ( C[ 7 ] ) );
//    return ( crc_read );
//}
//
//unsigned int Read_C( unsigned int index)
//{
//    unsigned int retval = 0;
//    if ( ( index >= 0) && ( index <= 7 ) )
//        retval = C[ index ];
//    return retval;
//}

uint32_t sensorMs5607ReadAdc(uint8_t aCMD)
{
  uint8_t value[3];
  uint32_t c=0;

//  send_cmd(MS5xxx_CMD_ADC_CONV+aCMD); // start DAQ and conversion of ADC data
  SensorI2C_writeReg(MS5xxx_CMD_ADC_CONV+aCMD, value, 0);
  switch (aCMD & 0x0f)
  {
  	//Sleep is based on clock ticks - set at 10uS in BIOS cfg file.
    case MS5xxx_CMD_ADC_256 : DELAY_US(900); //900 microS
    break;
    case MS5xxx_CMD_ADC_512 : DELAY_MS(3); //3 mS
    break;
    case MS5xxx_CMD_ADC_1024: DELAY_MS(4); //4 mS
    break;
    case MS5xxx_CMD_ADC_2048: DELAY_MS(6); //6 mS
    break;
    case MS5xxx_CMD_ADC_4096: DELAY_MS(10); //10 mS
    break;
  }
  SensorI2C_readReg(MS5xxx_CMD_ADC_READ, value, sizeof(value));
//  send_cmd(MS5xxx_CMD_ADC_READ); // read out values
//  _Wire->requestFrom(i2caddr, 3);
//  c = _Wire->read();
//  value = (c<<16);
//  c = _Wire->read();
//  value += (c<<8);
//  c = _Wire->read();
//  value += c;
//  _Wire->endTransmission(true);
  c = (value[0] << 16) | (value[1] << 8) | value[2];

  return c;
}

//void Readout() {
//	unsigned long D1=0, D2=0;
//
//	double dT;
//	double OFF;
//	double SENS;
//
//	D2=read_adc(MS5xxx_CMD_ADC_D2+MS5xxx_CMD_ADC_4096);
//	D1=read_adc(MS5xxx_CMD_ADC_D1+MS5xxx_CMD_ADC_4096);
//
//	// calculate 1st order pressure and temperature (MS5607 1st order algorithm)
//	dT=D2-C[5]*pow(2,8);
//	OFF=C[2]*pow(2,17)+dT*C[4]/pow(2,6);
//	SENS=C[1]*pow(2,16)+dT*C[3]/pow(2,7);
//	TEMP=(2000+(dT*C[6])/pow(2,23));
//	P=(((D1*SENS)/pow(2,21)-OFF)/pow(2,15));
//
//	// perform higher order corrections
//	double T2=0., OFF2=0., SENS2=0.;
//	if(TEMP<2000) {
//	  T2=dT*dT/pow(2,31);
//	  OFF2=61*(TEMP-2000)*(TEMP-2000)/pow(2,4);
//	  SENS2=2*(TEMP-2000)*(TEMP-2000);
//	  if(TEMP<-1500) {
//	    OFF2+=15*(TEMP+1500)*(TEMP+1500);
//	    SENS2+=8*(TEMP+1500)*(TEMP+1500);
//	  }
//	}
//
//	TEMP-=T2;
//	OFF-=OFF2;
//	SENS-=SENS2;
//	P=(((D1*SENS)/pow(2,21)-OFF)/pow(2,15));
//}
//
//double GetTemp() {
//	return TEMP;
//}
//
//double GetPres() {
//	return P;
//}
//
//unsigned char CRCcodeTest(){
//	unsigned int nprom[] = {0x3132,0x3334,0x3536,0x3738,0x3940,0x4142,0x4344,0x4500}; //expected output is 0xB
//	for(uint8_t i=0;i<8;i++) {
//		C[i] = nprom[i];
//	}
//	unsigned char crc = Calc_CRC4(); //expected output is 0xB
//	ReadProm();
//	return crc;
//}
