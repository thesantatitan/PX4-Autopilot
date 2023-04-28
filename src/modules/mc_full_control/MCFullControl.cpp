/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "MCFullControl.hpp"

MCFullControl::MCFullControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

MCFullControl::~MCFullControl()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool MCFullControl::init()
{
	// execute Run() on every sensor_accel publication
	if (!_sensor_accel_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_pos_control.setKx(0.8);_pos_control.setKv(0.8);_pos_control.setM(4.5);_pos_control.setKi(0.01);

	_att_control.setKr(1.0);_att_control.setKOmega(1.0);
	matrix::SquareMatrix3f inertia;
	inertia(0,0) = 0.029125;
	inertia(1,1) = 0.029125;
	inertia(2,2) = 0.055225;
	_att_control.setJ(inertia);

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate

	return true;
}

void MCFullControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}

	_trajectory_setpoint_sub.update(&_trajectory_setpoint);
	_pos_control.setSetpoint(_trajectory_setpoint);


	_vehicle_local_position_sub.update(&_vehicle_position);
	float dt = (_vehicle_position.timestamp_sample - _last_run)*1e-6f;
	_last_run = _vehicle_position.timestamp_sample;


	_pos_control.computeDesiredAttitude(
		matrix::Vector3f(_vehicle_position.x, _vehicle_position.y, _vehicle_position.z),
		matrix::Vector3f(_vehicle_position.vx, _vehicle_position.vy, _vehicle_position.vz),
		matrix::Vector3f(_vehicle_position.ax, _vehicle_position.ay, _vehicle_position.az),
		0.0f
	);

	// PX4_INFO("Acc: %f, %f, %f",(double)_vehicle_position.ax,(double)_vehicle_position.ay,(double)_vehicle_position.az);

	_att_control.setAttitudeSetpoint(_pos_control.getAttitudeSetpoint());
	_att_control.setOmegaSetpoint(_pos_control.getAngularVelocitySetpoint());
	_att_control.setOmegaSetpointDot(matrix::Vector3f(0.0f, 0.0f, 0.0f));


	_vehicle_attitude_sub.update(&_vehicle_attitude);
	_vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity);

	_att_control.update(matrix::Quatf(_vehicle_attitude.q), matrix::Vector3f(_vehicle_angular_velocity.xyz)).copyTo(_torque_setpoint.xyz);
	_thrust_setpoint.xyz[2] = _pos_control.updateThrustSetpoint(
		matrix::Vector3f(_vehicle_position.x, _vehicle_position.y, _vehicle_position.z),
		matrix::Vector3f(_vehicle_position.vx, _vehicle_position.vy, _vehicle_position.vz),
		matrix::Dcmf(matrix::Quatf(_vehicle_attitude.q)),
		dt
	);

	publishTorqueSetpoint(hrt_absolute_time());
	publishThrustSetpoint(hrt_absolute_time());
	publishActuatorControls(hrt_absolute_time());
	publishLocalPositionSetpoint(hrt_absolute_time());


	perf_end(_loop_perf);
}

void MCFullControl::publishTorqueSetpoint(const hrt_abstime &timestamp_sample)
{
	_torque_setpoint.timestamp_sample = timestamp_sample;
	_torque_setpoint.timestamp = hrt_absolute_time();
	_torque_setpoint.xyz[2] =  _torque_setpoint.xyz[2]>1.0f?1.0f:_torque_setpoint.xyz[2];
	_torque_setpoint.xyz[2] =  _torque_setpoint.xyz[2]<-1.0f?-1.0f:_torque_setpoint.xyz[2];
	_torque_setpoint.xyz[1] =  _torque_setpoint.xyz[1]>1.0f?1.0f:_torque_setpoint.xyz[1];
	_torque_setpoint.xyz[1] =  _torque_setpoint.xyz[1]<-1.0f?-1.0f:_torque_setpoint.xyz[1];
	_torque_setpoint.xyz[0] =  _torque_setpoint.xyz[0]>1.0f?1.0f:_torque_setpoint.xyz[0];
	_torque_setpoint.xyz[0] =  _torque_setpoint.xyz[0]<-1.0f?-1.0f:_torque_setpoint.xyz[0];
	// PX4_INFO("Torque: %f, %f, %f", (double)_torque_setpoint.xyz[0], (double)_torque_setpoint.xyz[1],
	// 	 (double)_torque_setpoint.xyz[2]);
	_vehicle_torque_setpoint_pub.publish(_torque_setpoint);
}

void MCFullControl::publishThrustSetpoint(const hrt_abstime &timestamp_sample)
{
	_thrust_setpoint.timestamp_sample = timestamp_sample;
	_thrust_setpoint.xyz[0] = 0.0;
	_thrust_setpoint.xyz[1] = 0.0;
	_thrust_setpoint.timestamp = hrt_absolute_time();
	_thrust_setpoint.xyz[2] =  _thrust_setpoint.xyz[2]>1.0f?1.0f:_thrust_setpoint.xyz[2];
	_thrust_setpoint.xyz[2] =  _thrust_setpoint.xyz[2]<-1.0f?-1.0f:_thrust_setpoint.xyz[2];
	// PX4_INFO("Thrust: %f",(double)_thrust_setpoint.xyz[2]);
	_vehicle_thrust_setpoint_pub.publish(_thrust_setpoint);
}

void MCFullControl::publishActuatorControls(const hrt_abstime &timestamp_sample)
{
	actuator_controls_s actuators{};
	actuators.control[actuator_controls_s::INDEX_ROLL] = _torque_setpoint.xyz[0];
	actuators.control[actuator_controls_s::INDEX_PITCH] = _torque_setpoint.xyz[1];
	actuators.control[actuator_controls_s::INDEX_YAW] = _torque_setpoint.xyz[2];
	actuators.control[actuator_controls_s::INDEX_THROTTLE] = -_thrust_setpoint.xyz[2];
	actuators.timestamp_sample = timestamp_sample;
	_actuator_controls_0_pub.publish(actuators);
}

void MCFullControl::publishLocalPositionSetpoint(const hrt_abstime &timestamp){
	vehicle_local_position_setpoint_s _local_sp{};
	_local_sp.timestamp = timestamp;
	_local_sp.x = _trajectory_setpoint.position[0];
	_local_sp.y = _trajectory_setpoint.position[1];
	_local_sp.z = _trajectory_setpoint.position[2];
	_local_sp.vx = _trajectory_setpoint.velocity[0];
	_local_sp.vy = _trajectory_setpoint.velocity[1];
	_local_sp.vz = _trajectory_setpoint.velocity[2];
	_vehicle_local_position_setpoint_pub.publish(_local_sp);
}

int MCFullControl::task_spawn(int argc, char *argv[])
{
	MCFullControl *instance = new MCFullControl();

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

int MCFullControl::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int MCFullControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MCFullControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_full_control_main(int argc, char *argv[])
{
	return MCFullControl::main(argc, argv);
}
