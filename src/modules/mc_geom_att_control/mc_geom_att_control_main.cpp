/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file mc_geom_att_control_main.cpp
 * Multicopter geometric attitude controller.
 *
 * @author Dev Rajput	<dev19034@iiitd.ac.in>
 *
 */

#include "mc_geom_att_control.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

MulticopterGeometricAttitudeControl::MulticopterGeometricAttitudeControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{

	parameters_updated();
}

MulticopterGeometricAttitudeControl::~MulticopterGeometricAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterGeometricAttitudeControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
MulticopterGeometricAttitudeControl::parameters_updated()
{
	_geometric_attitude_control.setKr(_param_mc_geom_att_kr.get());
	_geometric_attitude_control.setKOmega(_param_mc_geom_att_kom.get());
	_geometric_attitude_control.setJ(matrix::eye<float, 3>());
}

float
MulticopterGeometricAttitudeControl::throttle_curve(float throttle_stick_input)
{
	// throttle_stick_input is in range [0, 1]
	switch (_param_mpc_thr_curve.get()) {
	case 1: // no rescaling to hover throttle
		return math::interpolate(throttle_stick_input, 0.f, 1.f, _param_mpc_manthr_min.get(), _param_mpc_thr_max.get());

	default: // 0 or other: rescale to hover throttle at 0.5 stick
		return math::interpolateN(throttle_stick_input, {_param_mpc_manthr_min.get(), _param_mpc_thr_hover.get(), _param_mpc_thr_max.get()});
	}
}

void
MulticopterGeometricAttitudeControl::generate_attitude_setpoint(const Quatf &q, float dt, bool reset_yaw_sp)
{
	vehicle_attitude_setpoint_s attitude_setpoint{};
	const float yaw = Eulerf(q).psi();

	/* reset yaw setpoint to current position if needed */
	if (reset_yaw_sp) {
		_man_yaw_sp = yaw;

	} else if (math::constrain(_manual_control_setpoint.z, 0.0f, 1.0f) > 0.05f
		   || _param_mc_airmode.get() == 2) {

		const float yaw_rate = math::radians(_param_mpc_man_y_max.get());
		attitude_setpoint.yaw_sp_move_rate = _manual_control_setpoint.r * yaw_rate;
		_man_yaw_sp = wrap_pi(_man_yaw_sp + attitude_setpoint.yaw_sp_move_rate * dt);
	}

	/*
	 * Input mapping for roll & pitch setpoints
	 * ----------------------------------------
	 * We control the following 2 angles:
	 * - tilt angle, given by sqrt(x*x + y*y)
	 * - the direction of the maximum tilt in the XY-plane, which also defines the direction of the motion
	 *
	 * This allows a simple limitation of the tilt angle, the vehicle flies towards the direction that the stick
	 * points to, and changes of the stick input are linear.
	 */
	_man_x_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());
	_man_y_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());
	_man_x_input_filter.update(_manual_control_setpoint.x * _man_tilt_max);
	_man_y_input_filter.update(_manual_control_setpoint.y * _man_tilt_max);
	const float x = _man_x_input_filter.getState();
	const float y = _man_y_input_filter.getState();

	// we want to fly towards the direction of (x, y), so we use a perpendicular axis angle vector in the XY-plane
	Vector2f v = Vector2f(y, -x);
	float v_norm = v.norm(); // the norm of v defines the tilt angle

	if (v_norm > _man_tilt_max) { // limit to the configured maximum tilt angle
		v *= _man_tilt_max / v_norm;
	}

	Quatf q_sp_rpy = AxisAnglef(v(0), v(1), 0.f);
	Eulerf euler_sp = q_sp_rpy;
	attitude_setpoint.roll_body = euler_sp(0);
	attitude_setpoint.pitch_body = euler_sp(1);
	// The axis angle can change the yaw as well (noticeable at higher tilt angles).
	// This is the formula by how much the yaw changes:
	//   let a := tilt angle, b := atan(y/x) (direction of maximum tilt)
	//   yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2))).
	attitude_setpoint.yaw_body = _man_yaw_sp + euler_sp(2);


	/* copy quaternion setpoint to attitude setpoint topic */
	Quatf q_sp = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, attitude_setpoint.yaw_body);
	q_sp.copyTo(attitude_setpoint.q_d);

	attitude_setpoint.thrust_body[2] = -throttle_curve(math::constrain(_manual_control_setpoint.z, 0.f, 1.f));
	attitude_setpoint.timestamp = hrt_absolute_time();

	_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);

	// update attitude controller setpoint immediately
	_geometric_attitude_control.setAttitudeSetpoint(q_sp);
	_thrust_setpoint_body = Vector3f(attitude_setpoint.thrust_body);
	_last_attitude_setpoint = attitude_setpoint.timestamp;
}

void
MulticopterGeometricAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	// run controller on gyro updates
	vehicle_angular_velocity_s angular_velocity;
	// PX4_INFO("Outermost loop\n");
	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {
		// PX4_INFO("Inside if of attitude sub\n");
		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((angular_velocity.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
		_last_run = angular_velocity.timestamp_sample;


		_vehicle_attitude_sub.update(&v_att);

		const Quatf q{v_att.q};

		_manual_control_setpoint_sub.update(&_manual_control_setpoint);
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		// Check for new attitude setpoint
		if (_vehicle_attitude_setpoint_sub.updated()) {
			vehicle_attitude_setpoint_s vehicle_attitude_setpoint;
			vehicle_rates_setpoint_s vehicle_rates_setpoint;
			// PX4_INFO("Inside if of attitude setpoint sub\n");
			if (_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint)
			    && (vehicle_attitude_setpoint.timestamp > _last_attitude_setpoint)) {


				_vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint);

				_geometric_attitude_control.setAttitudeSetpoint(Quatf(vehicle_attitude_setpoint.q_d));
				_geometric_attitude_control.setOmegaSetpoint(Vector3f(vehicle_rates_setpoint.roll, vehicle_rates_setpoint.pitch, vehicle_rates_setpoint.yaw));
				_geometric_attitude_control.setOmegaSetpointDot(Vector3f(0.0f,0.0f,0.0f));
				_thrust_setpoint_body = Vector3f(vehicle_rates_setpoint.thrust_body);
				_last_attitude_setpoint = vehicle_attitude_setpoint.timestamp;
			}
		}

		/* check for updates in other topics */


		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
			}
		}

		bool attitude_setpoint_generated = false;


		bool run_att_ctrl = _vehicle_control_mode.flag_control_attitude_enabled || _vehicle_control_mode.flag_control_rates_enabled;
		if (run_att_ctrl) {
			// PX4_INFO("Inside if of attitude control sub\n");
			// Generate the attitude setpoint from stick inputs if we are in Manual/Stabilized mode
			if (_vehicle_control_mode.flag_control_manual_enabled &&
			    !_vehicle_control_mode.flag_control_altitude_enabled &&
			    !_vehicle_control_mode.flag_control_velocity_enabled &&
			    !_vehicle_control_mode.flag_control_position_enabled) {

				generate_attitude_setpoint(q, dt, _reset_yaw_sp);
				attitude_setpoint_generated = true;

			} else {
				_man_x_input_filter.reset(0.f);
				_man_y_input_filter.reset(0.f);
			}


			Vector3f torque_sp = _geometric_attitude_control.update(q,Vector3f(angular_velocity.xyz));
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};
			vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_torque_setpoint.xyz[0] = (PX4_ISFINITE(torque_sp(0))) ? torque_sp(0) : 0.0f;
			vehicle_torque_setpoint.xyz[1] = (PX4_ISFINITE(torque_sp(1))) ? torque_sp(1) : 0.0f;
			vehicle_torque_setpoint.xyz[2] = (PX4_ISFINITE(torque_sp(2))) ? torque_sp(2) : 0.0f;

			_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);

			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			_thrust_setpoint_body.copyTo(vehicle_thrust_setpoint.xyz);
			// PX4_INFO("Thrust: %f\n",(double)vehicle_thrust_setpoint.xyz[2]);
			_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

			actuator_controls_s actuators{};
			actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(torque_sp(0)) ? torque_sp(0) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(torque_sp(1)) ? torque_sp(1) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(torque_sp(2)) ? torque_sp(2) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_thrust_setpoint_body(2)) ? -_thrust_setpoint_body(
						2) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_LANDING_GEAR] = false;
			actuators.timestamp_sample = angular_velocity.timestamp_sample;

			actuators.timestamp = hrt_absolute_time();
			_actuator_controls_0_pub.publish(actuators);
		}

		// reset yaw setpoint during transitions, tailsitter.cpp generates
		// attitude setpoint for the transition
		_reset_yaw_sp = !attitude_setpoint_generated || _landed;
	}

	perf_end(_loop_perf);
}

int MulticopterGeometricAttitudeControl::task_spawn(int argc, char *argv[])
{
	MulticopterGeometricAttitudeControl *instance = new MulticopterGeometricAttitudeControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterGeometricAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterGeometricAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter attitude controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) as inputs and outputs a rate setpoint.

The controller has a P loop for angular error

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_geom_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


/**
 * Multicopter attitude control app start / stop handling function
 */
extern "C" __EXPORT int mc_geom_att_control_main(int argc, char *argv[])
{
	return MulticopterGeometricAttitudeControl::main(argc, argv);
}
