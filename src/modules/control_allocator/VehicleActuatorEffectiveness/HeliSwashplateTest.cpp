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

#include <gtest/gtest.h>
#include "HeliSwashplate.hpp"

static HeliSwashplate::Geometry makeStandard3ServoGeometry()
{
	HeliSwashplate::Geometry geo{};
	geo.count = 3;
	geo.servos[0] = {0.f, 1.f, 0.f};
	geo.servos[1] = {140.f, 1.f, 0.f};
	geo.servos[2] = {220.f, 1.f, 0.f};
	geo.max_servo_throw_deg = 0.f;
	return geo;
}

TEST(HeliSwashplateTest, ThreeServoCollectiveOnly)
{
	HeliSwashplate sp;
	sp.setGeometry(makeStandard3ServoGeometry());

	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp.setAll(0.f);

	const float collective = 0.5f;
	sp.mix(0.f, 0.f, collective, actuator_sp, 0);

	EXPECT_EQ(sp.count(), 3);

	for (int i = 0; i < 3; ++i) {
		EXPECT_FLOAT_EQ(actuator_sp(i), collective);
	}
}

TEST(HeliSwashplateTest, ThreeServoPurePitch)
{
	HeliSwashplate sp;
	sp.setGeometry(makeStandard3ServoGeometry());

	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp.setAll(0.f);

	const float pitch = 0.4f;
	sp.mix(0.f, pitch, 0.f, actuator_sp, 0);

	// For each servo: output = pitch * cos(angle_rad) * arm_length
	const float angles_deg[] = {0.f, 140.f, 220.f};

	for (int i = 0; i < 3; ++i) {
		const float angle_rad = math::radians(angles_deg[i]);
		const float pitch_coeff = cosf(angle_rad) * 1.f;
		const float expected = pitch * pitch_coeff;
		EXPECT_NEAR(actuator_sp(i), expected, 1e-6f);
	}
}

TEST(HeliSwashplateTest, ThreeServoPureRoll)
{
	HeliSwashplate sp;
	sp.setGeometry(makeStandard3ServoGeometry());

	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp.setAll(0.f);

	const float roll = 0.3f;
	sp.mix(roll, 0.f, 0.f, actuator_sp, 0);

	// For each servo: output = -roll * sin(angle_rad) * arm_length
	const float angles_deg[] = {0.f, 140.f, 220.f};

	for (int i = 0; i < 3; ++i) {
		const float angle_rad = math::radians(angles_deg[i]);
		const float roll_coeff = sinf(angle_rad) * 1.f;
		const float expected = -roll * roll_coeff;
		EXPECT_NEAR(actuator_sp(i), expected, 1e-6f);
	}
}

TEST(HeliSwashplateTest, FourServoMixing)
{
	HeliSwashplate::Geometry geo{};
	geo.count = 4;
	geo.servos[0] = {0.f, 1.f, 0.f};
	geo.servos[1] = {90.f, 1.f, 0.f};
	geo.servos[2] = {180.f, 1.f, 0.f};
	geo.servos[3] = {270.f, 1.f, 0.f};
	geo.max_servo_throw_deg = 0.f;

	HeliSwashplate sp;
	sp.setGeometry(geo);

	EXPECT_EQ(sp.count(), 4);

	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp.setAll(0.f);

	const float roll = 0.2f;
	const float pitch = 0.3f;
	const float collective = 0.5f;
	sp.mix(roll, pitch, collective, actuator_sp, 0);

	const float angles_deg[] = {0.f, 90.f, 180.f, 270.f};

	for (int i = 0; i < 4; ++i) {
		const float angle_rad = math::radians(angles_deg[i]);
		const float pitch_coeff = cosf(angle_rad) * 1.f;
		const float roll_coeff = sinf(angle_rad) * 1.f;
		const float expected = collective + pitch * pitch_coeff - roll * roll_coeff;
		EXPECT_NEAR(actuator_sp(i), expected, 1e-6f);
	}
}

TEST(HeliSwashplateTest, TrimOffsets)
{
	HeliSwashplate::Geometry geo{};
	geo.count = 3;
	geo.servos[0] = {0.f, 1.f, 0.1f};
	geo.servos[1] = {140.f, 1.f, -0.05f};
	geo.servos[2] = {220.f, 1.f, 0.2f};
	geo.max_servo_throw_deg = 0.f;

	HeliSwashplate sp;
	sp.setGeometry(geo);

	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp.setAll(0.f);

	const float collective = 0.5f;
	sp.mix(0.f, 0.f, collective, actuator_sp, 0);

	const float trims[] = {0.1f, -0.05f, 0.2f};

	for (int i = 0; i < 3; ++i) {
		EXPECT_NEAR(actuator_sp(i), collective + trims[i], 1e-6f);
	}
}

TEST(HeliSwashplateTest, StartIndexOffset)
{
	HeliSwashplate sp;
	sp.setGeometry(makeStandard3ServoGeometry());

	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp.setAll(0.f);

	const int start_index = 4;
	const float collective = 0.6f;
	sp.mix(0.f, 0.f, collective, actuator_sp, start_index);

	// Indices 0-3 must be untouched
	for (int i = 0; i < start_index; ++i) {
		EXPECT_FLOAT_EQ(actuator_sp(i), 0.f);
	}

	// Indices 4-6 should have the collective value
	for (int i = 0; i < 3; ++i) {
		EXPECT_FLOAT_EQ(actuator_sp(start_index + i), collective);
	}
}

TEST(HeliSwashplateTest, Linearization)
{
	const float max_throw_deg = 30.f;

	HeliSwashplate::Geometry geo_linear = makeStandard3ServoGeometry();
	geo_linear.max_servo_throw_deg = 0.f;

	HeliSwashplate::Geometry geo_nonlinear = makeStandard3ServoGeometry();
	geo_nonlinear.max_servo_throw_deg = max_throw_deg;

	HeliSwashplate sp_linear;
	sp_linear.setGeometry(geo_linear);

	HeliSwashplate sp_nonlinear;
	sp_nonlinear.setGeometry(geo_nonlinear);

	ActuatorEffectiveness::ActuatorVector act_linear{};
	ActuatorEffectiveness::ActuatorVector act_nonlinear{};
	act_linear.setAll(0.f);
	act_nonlinear.setAll(0.f);

	const float collective = 0.5f;
	sp_linear.mix(0.f, 0.f, collective, act_linear, 0);
	sp_nonlinear.mix(0.f, 0.f, collective, act_nonlinear, 0);

	// Verify linearized output matches the expected formula
	const float max_throw_rad = math::radians(max_throw_deg);
	const float max_servo_height = sinf(max_throw_rad);
	const float inverse_max_throw = 1.f / max_throw_rad;
	const float expected_linearized = inverse_max_throw * asinf(max_servo_height * collective);

	for (int i = 0; i < 3; ++i) {
		// Linear output should just be collective
		EXPECT_FLOAT_EQ(act_linear(i), collective);
		// Nonlinear output should differ from linear
		EXPECT_NE(act_nonlinear(i), act_linear(i));
		// Nonlinear output should match the asin formula
		EXPECT_NEAR(act_nonlinear(i), expected_linearized, 1e-6f);
	}
}
