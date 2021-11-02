/// \file Path.cpp
/// \brief Step and path generator for a single path motor
///
/// \copyright Written over 2014-2018 by Garth Zeglin <garthz@cmu.edu>.  To the
/// extent possible under law, the author has dedicated all copyright and
/// related and neighboring rights to this software to the public domain
/// worldwide.  This software is distributed without any warranty.  You should
/// have received a copy of the CC0 Public Domain Dedication along with this
/// software.  If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.

#include <Arduino.h>
#include <math.h>
#include <stdint.h>

#include "Path.h"

//================================================================
Path::Path()
{
  q    = 0.0;
  qd   = 0.0;
  qdd  = 0.0;
  q_d  = 0.0;
  qd_d = 0.0;

  q_d_d = 0.0;
  speed = INFINITY;

  t = 0;
  
  // Initialize the second-order model response to 2 Hz natural frequency, with
  // a damping ratio of 1.0 for critical damping.
  setFreqDamping(2.0, 1.0);
  
  qd_max  = 2400.0;     // typical physical limit for 4x microstepping
  qdd_max = 24000.0;
}

//================================================================
// Path integration running from main event loop.
void Path::pollForInterval(unsigned long interval)
{
  float dt = 1e-6 * interval;

  // calculate the derivatives
  float qdd = k * (q_d - q) + b * (qd_d - qd);

  // clamp the acceleration within range for safety
  qdd = constrain(qdd, -qdd_max, qdd_max);

  // integrate one time step
  q  += qd  * dt;
  qd += qdd * dt;
  t += dt;

  // clamp the model velocity within range for safety
  qd = constrain(qd, -qd_max, qd_max);

  // Update the reference trajectory using linear interpolation.  This can
  // create steps or ramps.  This calculates the maximum desired step, bounds it
  // to the speed, then applies the sign to move in the correct direction.
  float q_d_err = q_d_d - q_d;  // maximum error step

  if (q_d_err == 0.0) {
    qd_d = 0.0;  // make sure reference velocity is zero, leave reference position unchanged

  } else {
    if (isinf(speed)) {
      q_d = q_d_d;      // infinite speed, always adjust reference in one step
      qd_d = 0.0;       // then assume zero velocity
    } else {            // else calculate a ramp step
      float d_q_d_max = speed * dt; // maximum linear step, possibly infinite
      if (q_d_err > 0.0) {
	float d_q_d = min(d_q_d_max, q_d_err); // reference position step
	q_d += d_q_d;
	qd_d = speed; // reference velocity
      } else {
	float d_q_d = min(d_q_d_max, -q_d_err); // absolute value of reference position step
	q_d -= d_q_d;
	qd_d = -speed; // reference velocity
      }
    }
  }
}
//================================================================
