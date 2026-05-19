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

#include "HeliSwashplate.hpp"

void HeliSwashplate::setGeometry(const Geometry &geometry)
{
	_count = math::constrain(geometry.count, 2, NUM_SERVOS_MAX);

	for (int i = 0; i < _count; ++i) {
		const float angle_rad = math::radians(geometry.servos[i].angle_deg);
		_servos[i].pitch_coeff = cosf(angle_rad) * geometry.servos[i].arm_length;
		_servos[i].roll_coeff = sinf(angle_rad) * geometry.servos[i].arm_length;
		_servos[i].trim = geometry.servos[i].trim;
	}

	if (geometry.max_servo_throw_deg > 0.f) {
		_linearize = true;
		const float max_throw_rad = math::radians(geometry.max_servo_throw_deg);
		_max_servo_height = sinf(max_throw_rad);
		_inverse_max_servo_throw = 1.f / max_throw_rad;

	} else {
		_linearize = false;
		_max_servo_height = 0.f;
		_inverse_max_servo_throw = 0.f;
	}
}

void HeliSwashplate::mix(float roll, float pitch, float collective,
			 ActuatorEffectiveness::ActuatorVector &actuator_sp, int start_index) const
{
	for (int i = 0; i < _count; ++i) {
		float output = collective
			       + pitch * _servos[i].pitch_coeff
			       - roll * _servos[i].roll_coeff
			       + _servos[i].trim;

		if (_linearize) {
			output = math::constrain(output, -1.f, 1.f);
			output = _inverse_max_servo_throw * asinf(_max_servo_height * output);
		}

		actuator_sp(start_index + i) = output;
	}
}
