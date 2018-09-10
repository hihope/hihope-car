/* Includes ------------------------------------------------------------------*/ 

#ifndef __IMU_H
#define __IMU_H

#define AccSen							0.0078125	//g/lsb @ +/- 16g
#define GyroSen							0.01524		//¡ã/s/lsb @ 500
#define TempSen							0.5			//K/LSB center temperature is 23¡æ
#define MagxySen						0.3			//uT/lsb
#define MagzSen							0.15		//uT/lsb

//SDO1 SDO2 CSB3 pulled to GND
#define Acc_addr						0x18
#define Gyro_addr						0x68
#define Mag_addr						0x10


/* BMX055 Register Map */
//ACC define
#define	ACC_ID							0x00	//OXFA
#define	ACC_XL							0x02
#define	ACC_XM							0x03
#define	ACC_YL							0x04
#define	ACC_YM							0x05
#define	ACC_ZL							0x06
#define	ACC_ZM							0x07
#define	Temp							0x08
#define ACC_range						0x0f	//1100b --> +/- 16g
#define Shasow_dis						0x13
#define ACC_ret							0x14	//write 0xb6
//Gyro define
#define	GYRO_ID							0x00	//OXOF
#define	GYRO_XL							0x02
#define	GYRO_XM							0x03
#define	GYRO_YL							0x04
#define	GYRO_YM							0x05
#define	GYRO_ZL							0x06
#define	GYRO_ZM							0x07
#define GYRO_range						0x0f	//010b --> +/- 500¡ã/s
#define GYRO_ret						0x14	//write 0xb6
//MAG define
#define	MAG_ID							0x40	//OX32
#define	MAG_XL							0x42
#define	MAG_XM							0x43
#define	MAG_YL							0x44
#define	MAG_YM							0x45
#define	MAG_ZL							0x46
#define	MAG_ZM							0x47
#define	MAG_RHAL						0x48
#define	MAG_RHAM						0x49
#define MAG_ret							0x4b	//1000 0001b

/* Exported functions ------------------------------------------------------- */
#define PI 3.1416
#define MAX_CAPRATION_CIRCLE  4

extern int IMU_Init(void);
extern int imu_read_azimuth(void);
#endif
