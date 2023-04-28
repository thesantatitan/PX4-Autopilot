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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/geom_control/geom_att_control.hpp>
#include <lib/geom_control/geom_pos_control.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/actuator_controls.h>

using namespace time_literals;

class MCFullControl : public ModuleBase<MCFullControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	MCFullControl();
	~MCFullControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	void publishTorqueSetpoint(const hrt_abstime &timestamp_sample);
	void publishThrustSetpoint(const hrt_abstime &timestamp_sample);
	void publishActuatorControls(const hrt_abstime &timestamp_sample);

	float _last_run;

	GeometricAttitudeControl 		_att_control;
	GeometricPositionControl 		_pos_control;

	vehicle_local_position_s 		_vehicle_position;
	trajectory_setpoint_s			_trajectory_setpoint;

	vehicle_attitude_s 			_vehicle_attitude;
	vehicle_angular_velocity_s 		_vehicle_angular_velocity;
	vehicle_torque_setpoint_s		_torque_setpoint;
	vehicle_thrust_setpoint_s		_thrust_setpoint;

	// Publications
	uORB::Publication<orb_test_s> _orb_test_pub{ORB_ID(orb_test)};
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<actuator_controls_s>		_actuator_controls_0_pub{ORB_ID(actuator_controls_0)};

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};        // subscription that schedules MCFullControl when updated
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};          // regular subscription for additional data
	uORB::Subscription		   _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription		   _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription		   _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription		   _trajectory_setpoint_sub{(ORB_ID(trajectory_setpoint))};


	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)


	bool _armed{false};
};
