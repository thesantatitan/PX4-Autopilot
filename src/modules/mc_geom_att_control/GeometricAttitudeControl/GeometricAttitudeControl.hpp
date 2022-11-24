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

class GeometricAttitudeControl
{
public:
	GeometricAttitudeControl() = default;
	~GeometricAttitudeControl() = default;

	/**
	 * set the inertia matrix for the drone
	 */
	void setJ(matrix::SquareMatrix3f J){
		_J = J;
	}

	/**
	 * set proportianl gain for attitude
	*/
	void setKr(float kr){
		_kr = kr;
	}

	/**
	 * set proportinal gain for omega
	*/
	void setKOmega(float kOmega){
		_kOmega = kOmega;
	}

	void setAttitudeSetpoint(matrix::Quatf att_sp_q){
		_attitude_setpoint = matrix::Dcmf(att_sp_q);
	}

	void setOmegaSetpoint(matrix::Vector3f omega_sp){
		_omega_setpoint = omega_sp;
	}

	void setOmegaSetpointDot(matrix::Vector3f omega_sp_dot){
		_omega_setpoint_dot = omega_sp_dot;
	}

	matrix::Vector3f update(matrix::Quatf att_q, matrix::Vector3f omega);

private:
	float _kr;
	float _kOmega;
	matrix::SquareMatrix3f _J;
	matrix::Dcmf _attitude_setpoint;
	matrix::Vector3f _omega_setpoint;
	matrix::Vector3f _omega_setpoint_dot;
};
