/* 
 * ICM42607 sensor driver
 * Copyright (C) 2018 Invensense, Inc.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef ICM42607_GYRO_H
#define ICM42607_GYRO_H

/* ICM42607 sensor low power mode feature enable/disable */
#define ICM42607_GYRO_LOW_POWER_MODE

/*
FULL_DATA_RANGE 	65536 // 2^16

FULL_SCALE_RANGE1	500   // -250 ~ +250
FULL_SCALE_RANGE2	1000   // -500 ~ +500
FULL_SCALE_RANGE3	2000   // -1000 ~ +1000
FULL_SCALE_RANGE4	4000   // -2000 ~ +2000

SENSITIVITY		FULL_DATA_RANGE / FULL_SCALE_RANGE
!! SENSITIVITY accepted as same as SENSITIVITY_SCALE_FACTOR

Acquired Data (represented in FULL_DATA_RANGE) / SENSITIVITY_SCALE_FACTOR = Acquired DPS

Calculation of Current Sensitivy with FSR value (0 .. 3)
Current Sensitivity --> Max Sensitivity >> FSR Value

Radian to Degree Factor --> 180 / PI  : x radian = x * 180 / PI degree
Applying SENSITIVITY to above Radian to Degree Factor = 
	(180 / PI) * SENSITIVITY
ex) SENSITIVITY = 131
	(180 / PI) * 131 = 7506
	1 degree / 7506 = x radian

Measured data always scaled to MAX_SENSITIVITY,
so Degree to Radian Factor always be 7506 and it is a divider to measured data.


Orientation Translation
Mapping Sensor Orientation into Device (Phone) Orientation
Used when reading sensor data and calibration value
Device[AXIS] = Sensor[Converted AXIS] * Sign[AXIS]

Mapping Device (Phone) Orientation into Sensor Orientation
Used when writing calibration value
Sensor[Converted AXIS] = Device[AXIS] * Sign[AXIS]
*/

/* Max: ICM42607_GYRO_RANGE_250DPS */
#define ICM42607_GYRO_MAX_SENSITIVITY 131
/* Default: ICM42607_GYRO_RANGE_1000DPS */
#define ICM42607_GYRO_DEFAULT_SENSITIVITY 	328 /10 	// Default: ICM42607_GYRO_RANGE_1000DPS

/* the value calculated with MAX_SENSITIVITY */
#define DEGREE_TO_RAD 7506
#define ICM_GYRO_AXES_NUM 3

/*
    Full scale select for gyroscope UI interface output 
    b'000: ��2000dps (default) 
    b'001: ��1000dps 
    b'010: ��500dps 
    b'011: ��250dps 
*/
#define ICM42607_GYRO_RANGE_250DPS 3
#define ICM42607_GYRO_RANGE_500DPS 2
#define ICM42607_GYRO_RANGE_1000DPS 1
#define ICM42607_GYRO_RANGE_2000DPS 0
#endif /* ICM42607_GYRO_H */

