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

#include "ArmCompensator.hpp"

ArmCompensator::ArmCompensator(bool vtol) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint))

{
}

ArmCompensator::~ArmCompensator()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool ArmCompensator::init()
{
	// execute Run() on every sensor_accel publication
	// if (!_sensor_accel_sub.registerCallback()) {
	// 	PX4_ERR("callback registration failed");
	// 	return false;
	// }

	// alternatively, Run on fixed interval
	ScheduleOnInterval(2500_us); // 2000 us interval, 200 Hz rate
	// ScheduleNow();

	_m1 = _param_arm_comp_m1.get()/1000;
	_m2 = _param_arm_comp_m2.get()/1000;
	_l1 = _param_arm_comp_l1.get()/1000;
	_l2 = _param_arm_comp_l2.get()/1000;
	_mb = _param_arm_comp_mb.get()/1000;

	_lb = (_param_arm_comp_m1.get() * _param_arm_comp_l1.get() + _param_arm_comp_m2.get() *
			(_param_arm_comp_l1.get() + _param_arm_comp_l2.get()))/_param_arm_comp_mb.get();
	_lb = _lb/1000;

	// int i = 100;
	// while (i > 0)
	// {
	// 	PX4_INFO("lb: %f", (double) _lb);
	// 	i--;
	// }

	return true;
}


void ArmCompensator::update_comp_moments()
{
	float l1_sin_theta1 = (_l1 * sin(_theta1));
	float l1_cos_theta1 = (_l1 * cos(_theta1));
	float theta12 = _theta1 + _theta2;

	_Mx = _m1 * _param_arm_comp_g.get() * l1_sin_theta1 + _m2 * _param_arm_comp_g.get()
		 * (l1_sin_theta1 + _l2 * sin(theta12));

	_My = _param_arm_comp_g.get() * (_mb * _lb - _m1 * l1_cos_theta1
		- _m2 * (l1_cos_theta1 + _l2 * cos(theta12)));
}



void ArmCompensator::Run()
{
	if (should_exit()) {
		// ScheduleClear();
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

	// Example
	//  update vehicle_status to check arming state
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {

			const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

			if (armed && !_armed) {
				PX4_WARN("vehicle armed due to %d", vehicle_status.latest_arming_reason);

			} else if (!armed && _armed) {
				PX4_INFO("vehicle disarmed due to %d", vehicle_status.latest_disarming_reason);
			}

			_armed = armed;
		}
	}



	// Example
	//  grab latest accelerometer data
	if (_vehicle_torque_setpoint_sub.updated()) {
		vehicle_torque_setpoint_s tau;

		if (_vehicle_torque_setpoint_sub.copy(&tau)) {
			// DO WORK
			// PX4_INFO("hello");

			// access parameter value (SYS_AUTOSTART)
			if (_param_sys_autostart.get() == 1234) {
				// do something if SYS_AUTOSTART is 1234
			}
		}
	}


	// Example
	//  grab latest accelerometer data
	if (_vehicle_thrust_setpoint_sub.updated()) {
		vehicle_thrust_setpoint_s thrust;

		if (_vehicle_thrust_setpoint_sub.copy(&thrust)) {
			// DO WORK

			// access parameter value (SYS_AUTOSTART)
			if (_param_sys_autostart.get() == 1234) {
				// do something if SYS_AUTOSTART is 1234
			}
		}
	}

	// Example
	//  grab latest accelerometer data
	if (_input_rc_sub.updated()) {
		input_rc_s input;

		if (_input_rc_sub.copy(&input)) {
			// DO WORK
			// PX4_INFO("RC: %d", input.values[0]);

			// access parameter value (SYS_AUTOSTART)
			if (_param_sys_autostart.get() == 1234) {
				// do something if SYS_AUTOSTART is 1234
			}
		}
	}

	// PX4_INFO("RC input updated: %s", _input_rc_sub.updated() ? "true" : "false");

	// publish thrust and torque setpoints
			// vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			// _thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);
			// vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.f;
			// vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.f;
			// vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.f;

			vehicle_torque_setpoint.xyz[0] = 1.1f;
			vehicle_torque_setpoint.xyz[1] = 0.f;
			vehicle_torque_setpoint.xyz[2] = 0.f;



			// scale setpoints by battery status if enabled
			if (_param_mc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.f) {
					for (int i = 0; i < 3; i++) {
						// vehicle_thrust_setpoint.xyz[i] = math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
						vehicle_torque_setpoint.xyz[i] = math::constrain(vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
					}
				}
			}


			// vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);


	_theta1 = M_PI/6;
	_theta2 = M_PI/6;
	update_comp_moments();
	// PX4_INFO("Mx= %f", (double) _Mx);
	// PX4_INFO("My= %f", (double) _My);

	// Example
	//  publish some data
	orb_test_s data{};
	data.val = 314159;
	data.timestamp = hrt_absolute_time();
	_orb_test_pub.publish(data);
	// PX4_INFO("hello");


	perf_end(_loop_perf);
}

int ArmCompensator::task_spawn(int argc, char *argv[])
{
	ArmCompensator *instance = new ArmCompensator();

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

int ArmCompensator::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int ArmCompensator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ArmCompensator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("arm_compensator", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int arm_compensator_main(int argc, char *argv[])
{
	return ArmCompensator::main(argc, argv);
}
