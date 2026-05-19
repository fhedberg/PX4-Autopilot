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

#include "ActuatorEffectivenessHelicopterDual.hpp"
#include <lib/mathlib/mathlib.h>

using namespace matrix;
using namespace time_literals;

ActuatorEffectivenessHelicopterDual::ActuatorEffectivenessHelicopterDual(ModuleParams *parent, DualMode mode)
	: ModuleParams(parent), _mode(mode)
{
	// SP0 param handles (front/left rotor)
	for (int i = 0; i < NUM_SWASH_PLATE_SERVOS_MAX; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SP0_ANG%u", i);
		_param_handles.sp[0].angle[i] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SP0_ARM_L%u", i);
		_param_handles.sp[0].arm_length[i] = param_find(buffer);
	}

	_param_handles.sp[0].count = param_find("CA_SP0_COUNT");

	// SP1 param handles (rear/right rotor)
	for (int i = 0; i < NUM_SWASH_PLATE_SERVOS_MAX; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SP1_ANG%u", i);
		_param_handles.sp[1].angle[i] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SP1_ARM_L%u", i);
		_param_handles.sp[1].arm_length[i] = param_find(buffer);
	}

	_param_handles.sp[1].count = param_find("CA_SP1_COUNT");

	// Trim handles: CS0-CS3 for SP1, CS4-CS7 for SP2
	for (int i = 0; i < 2 * NUM_SWASH_PLATE_SERVOS_MAX; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TRIM", i);
		_param_handles.trim[i] = param_find(buffer);
	}

	// Throttle and pitch curves
	for (int i = 0; i < NUM_CURVE_POINTS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_HELI_THR_C%u", i);
		_param_handles.throttle_curve[i] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_HELI_PITCH_C%u", i);
		_param_handles.pitch_curve[i] = param_find(buffer);
	}

	// Dual-rotor specific params
	_param_handles.dcp_scaler = param_find("CA_HELI_DCP_SC");
	_param_handles.yaw_scaler = param_find("CA_HELI_YAW_SC");
	_param_handles.dual_engine = param_find("CA_HELI_DUAL_ENG");

	// Common params
	_param_handles.spoolup_time = param_find("COM_SPOOLUP_TIME");
	_param_handles.throttle_idle = param_find("CA_HELI_THR_IDLE");
	_param_handles.max_servo_throw = param_find("CA_MAX_SVO_THROW");

	updateParams();
}

void ActuatorEffectivenessHelicopterDual::updateParams()
{
	ModuleParams::updateParams();

	float max_servo_throw_deg = 0.f;
	param_get(_param_handles.max_servo_throw, &max_servo_throw_deg);

	// Load SP0 geometry
	int32_t sp0_count = 3;

	if (param_get(_param_handles.sp[0].count, &sp0_count) != PX4_OK) {
		PX4_ERR("param_get SP0 count failed");
	}

	_sp1_count = math::constrain((int)sp0_count, 3, NUM_SWASH_PLATE_SERVOS_MAX);

	HeliSwashplate::Geometry geo1{};
	geo1.count = _sp1_count;
	geo1.max_servo_throw_deg = max_servo_throw_deg;

	for (int i = 0; i < _sp1_count; ++i) {
		float angle_deg = 0.f;
		float arm_length = 1.f;
		float trim = 0.f;
		param_get(_param_handles.sp[0].angle[i], &angle_deg);
		param_get(_param_handles.sp[0].arm_length[i], &arm_length);
		param_get(_param_handles.trim[i], &trim);
		geo1.servos[i].angle_deg = angle_deg;
		geo1.servos[i].arm_length = arm_length;
		geo1.servos[i].trim = trim;
	}

	_swashplate1.setGeometry(geo1);

	// Load SP1 geometry
	int32_t sp1_count = 3;

	if (param_get(_param_handles.sp[1].count, &sp1_count) != PX4_OK) {
		PX4_ERR("param_get SP1 count failed");
	}

	_sp2_count = math::constrain((int)sp1_count, 3, NUM_SWASH_PLATE_SERVOS_MAX);

	HeliSwashplate::Geometry geo2{};
	geo2.count = _sp2_count;
	geo2.max_servo_throw_deg = max_servo_throw_deg;

	for (int i = 0; i < _sp2_count; ++i) {
		float angle_deg = 0.f;
		float arm_length = 1.f;
		float trim = 0.f;
		param_get(_param_handles.sp[1].angle[i], &angle_deg);
		param_get(_param_handles.sp[1].arm_length[i], &arm_length);
		param_get(_param_handles.trim[NUM_SWASH_PLATE_SERVOS_MAX + i], &trim);
		geo2.servos[i].angle_deg = angle_deg;
		geo2.servos[i].arm_length = arm_length;
		geo2.servos[i].trim = trim;
	}

	_swashplate2.setGeometry(geo2);

	// Load curves
	for (int i = 0; i < NUM_CURVE_POINTS; ++i) {
		param_get(_param_handles.throttle_curve[i], &_throttle_curve[i]);
		param_get(_param_handles.pitch_curve[i], &_pitch_curve[i]);
	}

	// Load dual params
	float dcp = 0.25f;
	param_get(_param_handles.dcp_scaler, &dcp);
	_dcp_scaler = dcp;

	float yaw_sc = 1.f;
	param_get(_param_handles.yaw_scaler, &yaw_sc);
	_yaw_scaler = yaw_sc;

	param_get(_param_handles.dual_engine, &_dual_engine);
	param_get(_param_handles.spoolup_time, &_spoolup_time);
	param_get(_param_handles.throttle_idle, &_throttle_idle);
}

bool ActuatorEffectivenessHelicopterDual::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// As the allocation is non-linear, we use updateSetpoint() instead of the matrix.
	// Register SP1 servos
	_first_sp1_servo_index = configuration.num_actuators_matrix[0];

	for (int i = 0; i < _sp1_count; ++i) {
		configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
	}

	// Register SP2 servos
	for (int i = 0; i < _sp2_count; ++i) {
		configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
	}

	// Throttle 1
	configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{});

	// Throttle 2 (only if dual engine)
	if (_dual_engine) {
		configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{});
	}

	return true;
}

void ActuatorEffectivenessHelicopterDual::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const ActuatorVector &actuator_min,
		const ActuatorVector &actuator_max)
{
	_saturation_flags = {};

	updateSpoolState();

	// Throttle/collective pitch curves
	const float commanded_throttle = math::interpolateN(-control_sp(ControlAxis::THRUST_Z), _throttle_curve);
	const float throttle = spoolupThrottle(commanded_throttle);
	const float collective = math::interpolateN(-control_sp(ControlAxis::THRUST_Z), _pitch_curve);

	const float roll = control_sp(ControlAxis::ROLL);
	const float pitch = control_sp(ControlAxis::PITCH);
	const float yaw = control_sp(ControlAxis::YAW);

	float roll1, pitch1, collective1;
	float roll2, pitch2, collective2;

	if (_mode == DualMode::TANDEM) {
		// Tandem (front/aft): roll same on both, pitch via DCP, yaw via differential lateral cyclic
		roll1 = roll;
		roll2 = roll;
		pitch1 = yaw * _yaw_scaler;
		pitch2 = -yaw * _yaw_scaler;
		collective1 = collective + pitch * _dcp_scaler;
		collective2 = collective - pitch * _dcp_scaler;

	} else { // TRANSVERSE
		// Transverse (side-by-side): pitch same on both, roll via DCP, yaw via differential longitudinal cyclic
		pitch1 = pitch;
		pitch2 = pitch;
		roll1 = yaw * _yaw_scaler;
		roll2 = -yaw * _yaw_scaler;
		collective1 = collective + roll * _dcp_scaler;
		collective2 = collective - roll * _dcp_scaler;
	}

	const int sp2_start = _first_sp1_servo_index + _sp1_count;
	_swashplate1.mix(roll1, pitch1, collective1, actuator_sp, _first_sp1_servo_index);
	_swashplate2.mix(roll2, pitch2, collective2, actuator_sp, sp2_start);

	const int throttle1_idx = sp2_start + _sp2_count;
	actuator_sp(throttle1_idx) = throttle;

	if (_dual_engine) {
		actuator_sp(throttle1_idx + 1) = throttle;

	} else {
		actuator_sp(throttle1_idx + 1) = NAN;
	}

	// Saturation checks for SP1 servos
	for (int i = 0; i < _sp1_count; i++) {
		const int idx = _first_sp1_servo_index + i;

		if (actuator_sp(idx) < actuator_min(idx)) {
			setSaturationFlag(-1.f, _saturation_flags.roll_pos, _saturation_flags.roll_neg);
			setSaturationFlag(-1.f, _saturation_flags.pitch_neg, _saturation_flags.pitch_pos);

		} else if (actuator_sp(idx) > actuator_max(idx)) {
			setSaturationFlag(-1.f, _saturation_flags.roll_neg, _saturation_flags.roll_pos);
			setSaturationFlag(-1.f, _saturation_flags.pitch_pos, _saturation_flags.pitch_neg);
		}
	}

	// Saturation checks for SP2 servos
	for (int i = 0; i < _sp2_count; i++) {
		const int idx = sp2_start + i;

		if (actuator_sp(idx) < actuator_min(idx)) {
			setSaturationFlag(-1.f, _saturation_flags.roll_pos, _saturation_flags.roll_neg);
			setSaturationFlag(-1.f, _saturation_flags.pitch_neg, _saturation_flags.pitch_pos);

		} else if (actuator_sp(idx) > actuator_max(idx)) {
			setSaturationFlag(-1.f, _saturation_flags.roll_neg, _saturation_flags.roll_pos);
			setSaturationFlag(-1.f, _saturation_flags.pitch_pos, _saturation_flags.pitch_neg);
		}
	}

	// Publish helicopter status
	helicopter_status_s heli_status{};
	heli_status.timestamp = hrt_absolute_time();
	heli_status.spool_state = static_cast<uint8_t>(_spool_state);
	heli_status.throttle_output = throttle;
	heli_status.spoolup_progress = (_spool_state == SpoolState::THROTTLE_UNLIMITED) ? 1.f :
				       (_spool_state == SpoolState::SPOOLING_UP) ?
				       math::constrain((hrt_absolute_time() - _armed_time) / 1e6f / _spoolup_time, 0.f, 1.f) :
				       0.f;
	_helicopter_status_pub.publish(heli_status);
}

void ActuatorEffectivenessHelicopterDual::updateSpoolState()
{
	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		const bool was_armed = _armed;
		_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
		_armed_time = vehicle_status.armed_time;

		if (_armed && !was_armed) {
			_spool_state = SpoolState::GROUND_IDLE;

		} else if (!_armed && was_armed) {
			_spool_state = SpoolState::SPOOLING_DOWN;
		}
	}

	if (!_armed) {
		if (_spool_state == SpoolState::SPOOLING_DOWN) {
			const float time_since_disarm = (hrt_absolute_time() - _armed_time) / 1e6f;

			if (time_since_disarm > _spoolup_time) {
				_spool_state = SpoolState::SHUT_DOWN;
			}

		} else {
			_spool_state = SpoolState::SHUT_DOWN;
		}

		return;
	}

	const float time_since_arming = (hrt_absolute_time() - _armed_time) / 1e6f;

	switch (_spool_state) {
	case SpoolState::SHUT_DOWN:
		_spool_state = SpoolState::GROUND_IDLE;
		break;

	case SpoolState::GROUND_IDLE:
		// Dual has no mainMotorEngaged check; go directly to spooling up
		_spool_state = SpoolState::SPOOLING_UP;
		break;

	case SpoolState::SPOOLING_UP:
		if (time_since_arming >= _spoolup_time) {
			_spool_state = SpoolState::THROTTLE_UNLIMITED;
		}

		break;

	case SpoolState::THROTTLE_UNLIMITED:
		break;

	case SpoolState::SPOOLING_DOWN:
		break;
	}
}

float ActuatorEffectivenessHelicopterDual::spoolupThrottle(float commanded_throttle)
{
	switch (_spool_state) {
	case SpoolState::SHUT_DOWN:
		return 0.f;

	case SpoolState::GROUND_IDLE:
		return _throttle_idle;

	case SpoolState::SPOOLING_UP: {
		const float time_since_arming = (hrt_absolute_time() - _armed_time) / 1e6f;
		const float progress = math::constrain(time_since_arming / _spoolup_time, 0.f, 1.f);
		return commanded_throttle * progress;
	}

	case SpoolState::THROTTLE_UNLIMITED:
		return commanded_throttle;

	case SpoolState::SPOOLING_DOWN:
		return _throttle_idle;
	}

	return 0.f;
}

void ActuatorEffectivenessHelicopterDual::setSaturationFlag(float coeff, bool &positive_flag, bool &negative_flag)
{
	if (coeff > 0.f) {
		positive_flag = true;

	} else if (coeff < 0.f) {
		negative_flag = true;
	}
}

void ActuatorEffectivenessHelicopterDual::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	// Note: the values '-1', '1' and '0' are just to indicate a negative,
	// positive or no saturation to the rate controller. The actual magnitude is not used.
	if (_saturation_flags.roll_pos) {
		status.unallocated_torque[0] = 1.f;

	} else if (_saturation_flags.roll_neg) {
		status.unallocated_torque[0] = -1.f;

	} else {
		status.unallocated_torque[0] = 0.f;
	}

	if (_saturation_flags.pitch_pos) {
		status.unallocated_torque[1] = 1.f;

	} else if (_saturation_flags.pitch_neg) {
		status.unallocated_torque[1] = -1.f;

	} else {
		status.unallocated_torque[1] = 0.f;
	}

	if (_saturation_flags.yaw_pos) {
		status.unallocated_torque[2] = 1.f;

	} else if (_saturation_flags.yaw_neg) {
		status.unallocated_torque[2] = -1.f;

	} else {
		status.unallocated_torque[2] = 0.f;
	}

	if (_saturation_flags.thrust_pos) {
		status.unallocated_thrust[2] = 1.f;

	} else if (_saturation_flags.thrust_neg) {
		status.unallocated_thrust[2] = -1.f;

	} else {
		status.unallocated_thrust[2] = 0.f;
	}
}
