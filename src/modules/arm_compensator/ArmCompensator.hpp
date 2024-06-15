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

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/robotic_arm_moments.h>

using namespace time_literals;

class ArmCompensator : public ModuleBase<ArmCompensator>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	ArmCompensator(bool vtol = false);
	~ArmCompensator() override;

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

	void update_rc_inputs_and_thetas();

	void update_comp_moments();

	void publish_arm_moments();

	void run_diagnostics();

	void run_rc_input_diagnostics();

	void parameters_updated();


	float mapValue(int x, int in_min, int in_max, float out_min, float out_max);

	// Publications
	uORB::Publication<orb_test_s> _orb_test_pub{ORB_ID(orb_test)};

	// Subscriptions
	// uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};        // subscription that schedules WorkItemExample when updated
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};          // regular subscription for additional data
	uORB::Subscription 		   _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription 		   _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription 		   _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription 		   _vehicle_torque_setpoint_sub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Subscription 		   _vehicle_thrust_setpoint_sub{ORB_ID(vehicle_thrust_setpoint)};



	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub;
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub;
	uORB::Publication<robotic_arm_moments_s>	_robotic_arm_moments_pub;


	float _battery_status_scale{0.0f};
	float _m1{0.0f}, _m2{0.0f}, _l1{0.0f}, _l2{0.0f};
	float _mb{0.0f}, _lb{0.0f};
	float _Mx{0.0f}, _My{0.0f};
	float _theta1{0.0f}, _theta2{0.0f};

	// RC inputs
	int16_t _input_rc_theta1{1500}, _input_rc_theta2{1500};


	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) 			_param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) 		_param_sys_autoconfig,  /**< another parameter */
		(ParamBool<px4::params::MC_BAT_SCALE_EN>) 		_param_mc_bat_scale_en,
		(ParamBool<px4::params::ARM_COMP_ENABLE>) 		_param_arm_comp_enable,
		(ParamFloat<px4::params::ARM_COMP_M1>) 			_param_arm_comp_m1,
		(ParamFloat<px4::params::ARM_COMP_M2>) 			_param_arm_comp_m2,
		(ParamFloat<px4::params::ARM_COMP_L1>) 			_param_arm_comp_l1,
		(ParamFloat<px4::params::ARM_COMP_L2>) 			_param_arm_comp_l2,
		(ParamFloat<px4::params::ARM_COMP_MB>) 			_param_arm_comp_mb,
		(ParamFloat<px4::params::ARM_COMP_G>) 			_param_arm_comp_g,
		(ParamFloat<px4::params::ARM_COMP_FACTOR>) 		_param_arm_comp_factor,
		(ParamInt<px4::params::ARM_COMP_FLIP_MX>) 		_param_arm_comp_flip_mx,
		(ParamInt<px4::params::ARM_COMP_FLIP_MY>) 		_param_arm_comp_flip_my,
		(ParamInt<px4::params::ARM_COMP_CH_T1>) 		_param_arm_comp_ch_theta1,
		(ParamInt<px4::params::ARM_COMP_CH_T2>) 		_param_arm_comp_ch_theta2
	)


	bool _armed{false};
};
