/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file arm_compensator_params.c
 *
 * Parameters for arm compensator
 */

/**
 * Acceleration due to gravity g
 *
 * @min 0.01
 * @max 100
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ARM_COMP_G, 9.81f);

/**
 * Mass m1 in grams
 *
 * @min 0.01
 * @max 1000
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ARM_COMP_M1, 100.0f);

/**
 * Mass m2 in grams
 *
 * @min 0.01
 * @max 1000
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ARM_COMP_M2, 150.0f);

/**
 * arm length l1 in mm
 *
 * @min 0.01
 * @max 2000
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ARM_COMP_L1, 345.5f);

/**
 * arm length l2 in mm
 *
 * @min 0.01
 * @max 2000
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ARM_COMP_L2, 462.5f);

/**
 * Body mass mb in grams
 *
 * @min 0.01
 * @max 100000
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ARM_COMP_MB, 1200.0f);


/**
 * Moment divider
 *
 * @min 0.01
 * @max 100000
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ARM_COMP_FACTOR, 10.0f);


/**
 * Reverse the compensated roll moment
 *
 * @min -1
 * @max 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(ARM_COMP_FLIP_MX, 1);

/**
 * Reverse the compensated pitch moment
 *
 * @min -1
 * @max 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(ARM_COMP_FLIP_MY, 1);


/**
 * Enable robotic arm moment compensation
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(ARM_COMP_ENABLE, 0);
