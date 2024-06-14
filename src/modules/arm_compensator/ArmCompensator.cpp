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
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	_robotic_arm_moments_pub(vtol ? ORB_ID(robotic_arm_moments_virtual_mc) : ORB_ID(robotic_arm_moments))
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
	ScheduleOnInterval(2000_us); // 2000 us interval, 500 Hz rate max
	// ScheduleNow();

	parameters_updated();

	// int i = 100;
	// while (i > 0)
	// {
	// 	PX4_INFO("lb: %f", (double) _lb);
	// 	i--;
	// }

	// run_diagnostics();

	return true;
}


void ArmCompensator::parameters_updated()
{
	// Reinitialize parameters after updating
        _m1 = _param_arm_comp_m1.get() / 1000;
        _m2 = _param_arm_comp_m2.get() / 1000;
        _l1 = _param_arm_comp_l1.get() / 1000;
        _l2 = _param_arm_comp_l2.get() / 1000;
        _mb = _param_arm_comp_mb.get() / 1000;

        _lb = (_param_arm_comp_m1.get() * _param_arm_comp_l1.get() + _param_arm_comp_m2.get() *
               (_param_arm_comp_l1.get() + _param_arm_comp_l2.get())) / _param_arm_comp_mb.get();
        _lb = _lb / 1000;
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
		parameters_updated();
	}


	update_rc_inputs_and_thetas();

	// _theta1 = M_PI/6;
	// _theta2 = M_PI/6;
	update_comp_moments();
	// PX4_INFO("Mx= %f", (double) _Mx);
	// PX4_INFO("My= %f", (double) _My);

	publish_arm_moments();


	perf_end(_loop_perf);
}


void ArmCompensator::update_rc_inputs_and_thetas()
{
	if (_input_rc_sub.updated()) {
		input_rc_s input;

		if (_input_rc_sub.copy(&input)) {

			_input_rc_theta1 = input.values[_param_arm_comp_ch_theta1.get()];
			_input_rc_theta2 = input.values[_param_arm_comp_ch_theta2.get()];

			// PX4_INFO("RC: %d", input.values[_param_arm_comp_ch_theta1.get()]);
		}
	}


	const float theta_min = -M_PI / 6;
	const float theta_max = M_PI / 6;

	_theta1 = mapValue(_input_rc_theta1, 1000, 2000, theta_min, theta_max);
	_theta2 = mapValue(_input_rc_theta2, 1000, 2000, theta_min, theta_max);

	// Print mapped values for debugging
	// PX4_INFO("Mapped _theta1: %f", (double) math::degrees(_theta1));
	// PX4_INFO("Mapped _theta2: %f", (double) math::degrees(_theta2));

}

float ArmCompensator::mapValue(int x, int in_min, int in_max, float out_min, float out_max)
{
    return (float(x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}



void ArmCompensator::publish_arm_moments()
{
	robotic_arm_moments_s moments{};

	moments.xyz[0] = _param_arm_comp_flip_mx.get() * (_Mx/_param_arm_comp_factor.get()) * _param_arm_comp_enable.get();
	moments.xyz[1] = _param_arm_comp_flip_my.get() * (_My/_param_arm_comp_factor.get()) * _param_arm_comp_enable.get();
	moments.xyz[2] = 0.f;
	moments.timestamp = hrt_absolute_time();
	_robotic_arm_moments_pub.publish(moments);
}



void ArmCompensator::update_comp_moments()
{
	float l1_sin_theta1 = (_l1 * sinf(_theta1));
	float l1_cos_theta1 = (_l1 * cosf(_theta1));
	float theta12 = _theta1 + _theta2;

	_Mx = _m1 * _param_arm_comp_g.get() * l1_sin_theta1 + _m2 * _param_arm_comp_g.get()
		 * (l1_sin_theta1 + _l2 * sinf(theta12));

	_My = _param_arm_comp_g.get() * (_mb * _lb - _m1 * l1_cos_theta1
		- _m2 * (l1_cos_theta1 + _l2 * cosf(theta12)));
}



void ArmCompensator::run_diagnostics()
{
    parameters_updated();

    // Check if parameters are initialized correctly
    if (_m1 <= 0 || _m2 <= 0 || _l1 <= 0 || _l2 <= 0 || _mb <= 0) {
        PX4_ERR("Parameter initialization failed.");
        return;
    }

    // Perform a sample computation and log the results
    float test_theta1 = M_PI / 6;
    float test_theta2 = M_PI / 6;

    float test_Mx = _m1 * _param_arm_comp_g.get() * (_l1 * sinf(test_theta1)) +
                    _m2 * _param_arm_comp_g.get() * (_l1 * sinf(test_theta1) + _l2 * sinf(test_theta1 + test_theta2));

    float test_My = _param_arm_comp_g.get() * (_mb * _lb - _m1 * _l1 * cosf(test_theta1) -
                    _m2 * (_l1 * cosf(test_theta1) + _l2 * cosf(test_theta1 + test_theta2)));

//     test_Mx -= 1.013059f;
//     test_My -= 0.453806f;

    PX4_INFO("Diagnostic Test: Mx = %f, My = %f", (double)test_Mx, (double)test_My);


    // Perform additional checks if necessary
    if (fabs(test_Mx - 1.013059f) < 0.000001f && fabs(test_My - 0.453806f) < 0.000001f) {
        PX4_INFO("Diagnostic Test Passed.");
    } else {
        PX4_ERR("Diagnostic Test Failed.");
    }
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
    if (!strcmp(argv[0], "diagnostics")) {
        ArmCompensator instance;
        instance.run_diagnostics();
        return 0;
    }
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
The `arm_compensator` module calculates compensatory moments for a robotic arm mounted on a vehicle.

This module reads RC input values to set the angles (_theta1 and _theta2) of the robotic arm joints. Using these angles and predefined arm parameters, it computes the compensatory moments needed to counteract the forces generated by the arm's movement.

Key features:
- Reads RC input channels for joint angles.
- Maps RC input values to angles in radians.
- Calculates compensatory moments (_Mx and _My) based on the arm's configuration and gravity.
- Publishes the calculated moments to a dedicated topic.

Parameters:
- `ARM_COMP_M1`, `ARM_COMP_M2`: Masses of the arm segments.
- `ARM_COMP_L1`, `ARM_COMP_L2`: Lengths of the arm segments.
- `ARM_COMP_MB`: Mass of the base segment.
- `ARM_COMP_G`: Gravity constant.
- `ARM_COMP_CH_THETA1`, `ARM_COMP_CH_THETA2`: RC input channels for the arm angles.
- `ARM_COMP_FACTOR`, `ARM_COMP_FLIP_MX`, `ARM_COMP_FLIP_MY`: Factors for adjusting the published moments.
- `ARM_COMP_ENABLE`: Enable/disable the moment compensation.

### Implementation Details
- Scheduled to run at 500 Hz max.
- Utilizes the work queue configuration `rate_ctrl`.
- Updates parameters dynamically when they change.

### Usage
- `start`: Start the module.
- `diagnostics`: Run diagnostic tests to ensure the module is functioning correctly.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("arm_compensator", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("diagnostics");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int arm_compensator_main(int argc, char *argv[])
{
	return ArmCompensator::main(argc, argv);
}
