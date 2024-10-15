//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef __MADGWICK_AHRS_H
#define __MADGWICK_AHRS_H

#include <math.h>

//--------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
// Function declarations
void madgwick_init(float sampleFrequency);
void madgwick_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void madgwick_update_imu(float gx, float gy, float gz, float ax, float ay, float az);

float madgwick_get_roll(void);
float madgwick_get_pitch(void);
float madgwick_get_yaw(void);
float madgwick_get_roll_radians(void);
float madgwick_get_pitch_radians(void);
float madgwick_get_yaw_radians(void);

#endif // __MADGWICK_AHRS_H
