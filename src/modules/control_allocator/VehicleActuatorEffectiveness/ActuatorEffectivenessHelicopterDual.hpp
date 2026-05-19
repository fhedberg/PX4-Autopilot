/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include "HeliSwashplate.hpp"

#include <px4_platform_common/module_params.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/helicopter_status.h>

class ActuatorEffectivenessHelicopterDual : public ModuleParams, public ActuatorEffectiveness
{
public:
	enum class DualMode : int32_t {
		TANDEM = 0,
		TRANSVERSE = 1,
	};

	static constexpr int NUM_SWASH_PLATE_SERVOS_MAX = 4;
	static constexpr int NUM_CURVE_POINTS = 5;

	ActuatorEffectivenessHelicopterDual(ModuleParams *parent, DualMode mode);
	virtual ~ActuatorEffectivenessHelicopterDual() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;
	const char *name() const override { return "Helicopter Dual"; }
	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index, ActuatorVector &actuator_sp,
			    const ActuatorVector &actuator_min, const ActuatorVector &actuator_max) override;
	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

private:
	void updateParams() override;
	void updateSpoolState();
	float spoolupThrottle(float commanded_throttle);

	enum class SpoolState : uint8_t {
		SHUT_DOWN = 0,
		GROUND_IDLE,
		SPOOLING_UP,
		THROTTLE_UNLIMITED,
		SPOOLING_DOWN,
	};

	struct SaturationFlags {
		bool roll_pos;
		bool roll_neg;
		bool pitch_pos;
		bool pitch_neg;
		bool yaw_pos;
		bool yaw_neg;
		bool thrust_pos;
		bool thrust_neg;
	};

	static void setSaturationFlag(float coeff, bool &positive_flag, bool &negative_flag);

	struct SwashPlateParamHandles {
		param_t angle[NUM_SWASH_PLATE_SERVOS_MAX];
		param_t arm_length[NUM_SWASH_PLATE_SERVOS_MAX];
		param_t count;
	};

	struct ParamHandles {
		SwashPlateParamHandles sp[2];
		param_t trim[2 * NUM_SWASH_PLATE_SERVOS_MAX];
		param_t throttle_curve[NUM_CURVE_POINTS];
		param_t pitch_curve[NUM_CURVE_POINTS];
		param_t dcp_scaler;
		param_t yaw_scaler;
		param_t dual_engine;
		param_t spoolup_time;
		param_t throttle_idle;
		param_t max_servo_throw;
	};

	const DualMode _mode;
	ParamHandles _param_handles{};

	HeliSwashplate _swashplate1;
	HeliSwashplate _swashplate2;
	int _sp1_count{3};
	int _sp2_count{3};

	float _throttle_curve[NUM_CURVE_POINTS]{};
	float _pitch_curve[NUM_CURVE_POINTS]{};
	float _dcp_scaler{0.25f};
	float _yaw_scaler{1.f};
	int32_t _dual_engine{0};
	float _spoolup_time{10.f};
	float _throttle_idle{0.05f};

	int _first_sp1_servo_index{0};
	SaturationFlags _saturation_flags{};

	SpoolState _spool_state{SpoolState::SHUT_DOWN};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Publication<helicopter_status_s> _helicopter_status_pub{ORB_ID(helicopter_status)};
	bool _armed{false};
	uint64_t _armed_time{0};
};
