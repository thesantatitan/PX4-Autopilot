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
 * @file GeometricAttitudeControl.hpp
 *
 * A geometric attitude controller.
 *
 * @author Dev Rajput	<dev19034@iiitd.ac.in>
 *
 * A geometric controller based on the publication
 * https://arxiv.org/pdf/1003.2005.pdf
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>
#include <uORB/topics/vehicle_local_position_setpoint.h>

class GeometricPositionControl
{
public:
	GeometricPositionControl() = default;
	~GeometricPositionControl() = default;

	void setKx(float _kx){
		kx = _kx;
	}

	void setKv(float _kv){
		kv = _kv;
	}

	void setM(float _m){
		m = _m;
	}

	void setSetpoint(vehicle_local_position_setpoint_s _setpoint){
		position_setpoint(0) = _setpoint.x;position_setpoint(1) = _setpoint.y;position_setpoint(2) = _setpoint.z;
		velocity_setpoint(0) = _setpoint.vx;velocity_setpoint(1) = _setpoint.vy;velocity_setpoint(2) = _setpoint.vz;
		acceleration_setpoint(0) = _setpoint.acceleration[0];acceleration_setpoint(1) = _setpoint.acceleration[1];acceleration_setpoint(2) = _setpoint.acceleration[2];
	}

	void setPositionSetpoint(matrix::Vector3f _position_setpoint){
		position_setpoint = _position_setpoint;
	}

	void setVelocitySetpoint(matrix::Vector3f _velocity_setpoint){
		velocity_setpoint = _velocity_setpoint;
	}

	void setAccelerationSetpoint(matrix::Vector3f _acceleration_setpoint){
		acceleration_setpoint = _acceleration_setpoint;
	}

	matrix::Quatf getAttitudeSetpoint(){
		return matrix::Quatf(Rc);
	}

	matrix::Vector3f getAngularVelocitySetpoint(){
		return matrix::Vector3f(angular_velocity_setpoint);
	}

	void computeDesiredAttitude(matrix::Vector3f position, matrix::Vector3f velocity, matrix::Vector3f acceleration, float yaw);

	float updateThrustSetpoint(matrix::Vector3f position, matrix::Vector3f velocity, matrix::Dcmf attitude);

private:
	float kx;
	float kv;
	float m;
	matrix::Vector3f position_setpoint;
	matrix::Vector3f velocity_setpoint;
	matrix::Vector3f acceleration_setpoint;
	matrix::Vector3f e3 = matrix::Vector3f({0,0,1});
	matrix::Dcmf Rc;
	matrix::Vector3f angular_velocity_setpoint;
};
