/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file GeometricAttitudeControl.cpp
 */

#include <geom_pos_control.hpp>

#include <mathlib/math/Functions.hpp>
#include <px4_log.h>

void GeometricPositionControl::computeDesiredAttitude(matrix::Vector3f position, matrix::Vector3f velocity, matrix::Vector3f acceleration, float yaw){
	matrix::Vector3f ex = position - position_setpoint;
	matrix::Vector3f ev = velocity - velocity_setpoint;
	matrix::Vector3f ea = acceleration - acceleration_setpoint;

	matrix::Vector3f A = kx*ex + kv*ev + m*9.81f*e3 - m*acceleration_setpoint;
	matrix::Vector3f A_dot = kx*ev + kv*ea;

	float A_norm = A.norm();
	matrix::Vector3f b3c = A.unit();
	matrix::Vector3f b3c_dot = A_dot/A_norm - A*A.dot(A_dot)/(A_norm*A_norm*A_norm);

	matrix::Vector3f b1d(cos(yaw), sin(yaw), 0.0f);
	matrix::Vector3f b1c = b1d - (b1d.dot(b3c))*b3c; //Since b3c is already a unit vector, no need to normalize it here again
	matrix::Vector3f b2c = b3c.cross(b1c);

	matrix::Vector3f b1d_dot(-sin(yaw), cos(yaw), 0.0f);
	matrix::Vector3f b1c_dot = b1d_dot - b1d_dot.dot(b3c)*b3c - b1d.dot(b3c_dot)*b3c - b1d.dot(b3c)*b3c_dot;
	matrix::Vector3f b2c_dot = b3c_dot.cross(b1c_dot);

	{
		float temp_mat[3][3] = {{b1c(0), b2c(0), b3c(0)}, {b1c(1), b2c(1), b3c(1)}, {b1c(2), b2c(2), b3c(2)}};
		Rc = matrix::Dcmf(temp_mat);
	}
	matrix::Eulerf euler(Rc);
	// PX4_INFO("R,P,Y: %f, %f,%f",(double)euler(0)*180/3.14, (double)euler(1)*180/3.14, (double)euler(2)*180/3.14);
	matrix::Dcmf Rc_dot;
	{
		float temp_mat[3][3] = {{b1c_dot(0), b2c_dot(0), b3c_dot(0)}, {b1c_dot(1), b2c_dot(1), b3c_dot(1)}, {b1c_dot(2), b2c_dot(2), b3c_dot(2)}};
		Rc_dot = matrix::Dcmf(temp_mat);
	}
	matrix::Matrix3f omega_hat = Rc.transpose()*Rc_dot;
	angular_velocity_setpoint(0) = omega_hat(2,1);
	angular_velocity_setpoint(1) = omega_hat(0,2);
	angular_velocity_setpoint(2) = omega_hat(1,0);
}


float GeometricPositionControl::updateThrustSetpoint(matrix::Vector3f position, matrix::Vector3f velocity, matrix::Dcmf attitude){
	matrix::Vector3f ex = position - position_setpoint;
	matrix::Vector3f ev = velocity - velocity_setpoint;

	// PX4_INFO("EX: %f, %f, %f",(double)ex(0),(double)ex(1),(double)ex(2));
	// PX4_INFO("EV: %f, %f, %f",(double)ev(0),(double)ev(1),(double)ev(2));
	matrix::Vector3f temp = (kx*ex + kv*ev + m*9.81f*e3 - m*acceleration_setpoint);
	return -temp.dot(attitude*e3)*0.7f/(m*9.81f);
}

