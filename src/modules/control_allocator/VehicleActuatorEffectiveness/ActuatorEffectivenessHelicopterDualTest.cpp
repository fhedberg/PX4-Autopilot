/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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
#include "ActuatorEffectivenessHelicopterDual.hpp"

using namespace matrix;

// Standard 3-servo geometry: 0/140/220 degrees, arm_length=1
// Servo 0 (0 deg):   pitch_coeff=1.0,     roll_coeff=0.0
// Servo 1 (140 deg): pitch_coeff=-0.766.., roll_coeff=0.643..
// Servo 2 (220 deg): pitch_coeff=-0.766.., roll_coeff=-0.643..
//
// HeliSwashplate::mix() computes:
//   output = collective + pitch * pitch_coeff - roll * roll_coeff + trim

// Helper: set up standard 3-servo geometry for both swashplates, linear curves,
// and the specified DCP/yaw scalers.
static void setupDualParams(float dcp_scaler, float yaw_scaler, int dual_engine)
{
	param_control_autosave(false);

	// SP0 geometry (3 servos: 0/140/220 deg, arm_length=1)
	float angles0[] = {0.f, 140.f, 220.f};
	float arm_len = 1.f;

	for (int i = 0; i < 3; ++i) {
		char buf[17];
		snprintf(buf, sizeof(buf), "CA_SP0_ANG%u", i);
		param_set(param_find(buf), &angles0[i]);
		snprintf(buf, sizeof(buf), "CA_SP0_ARM_L%u", i);
		param_set(param_find(buf), &arm_len);
	}

	int32_t count3 = 3;
	param_set(param_find("CA_SP0_COUNT"), &count3);

	// SP1 geometry (same as SP0)
	float angles1[] = {0.f, 140.f, 220.f};

	for (int i = 0; i < 3; ++i) {
		char buf[17];
		snprintf(buf, sizeof(buf), "CA_SP1_ANG%u", i);
		param_set(param_find(buf), &angles1[i]);
		snprintf(buf, sizeof(buf), "CA_SP1_ARM_L%u", i);
		param_set(param_find(buf), &arm_len);
	}

	param_set(param_find("CA_SP1_COUNT"), &count3);

	// Zero trims for all 8 servo channels
	float zero = 0.f;

	for (int i = 0; i < 8; ++i) {
		char buf[17];
		snprintf(buf, sizeof(buf), "CA_SV_CS%u_TRIM", i);
		param_set(param_find(buf), &zero);
	}

	// Linear throttle curve: 0, 0.25, 0.5, 0.75, 1.0
	float thr_curve[] = {0.f, 0.25f, 0.5f, 0.75f, 1.f};

	for (int i = 0; i < 5; ++i) {
		char buf[17];
		snprintf(buf, sizeof(buf), "CA_HELI_THR_C%u", i);
		param_set(param_find(buf), &thr_curve[i]);
	}

	// Linear pitch curve: -0.5, -0.25, 0, 0.25, 0.5
	float pitch_curve[] = {-0.5f, -0.25f, 0.f, 0.25f, 0.5f};

	for (int i = 0; i < 5; ++i) {
		char buf[17];
		snprintf(buf, sizeof(buf), "CA_HELI_PITCH_C%u", i);
		param_set(param_find(buf), &pitch_curve[i]);
	}

	// Dual params
	param_set(param_find("CA_HELI_DCP_SC"), &dcp_scaler);
	param_set(param_find("CA_HELI_YAW_SC"), &yaw_scaler);

	int32_t de = dual_engine;
	param_set(param_find("CA_HELI_DUAL_ENG"), &de);

	// Common params
	float spoolup = 10.f;
	param_set(param_find("COM_SPOOLUP_TIME"), &spoolup);

	float idle = 0.f;
	param_set(param_find("CA_HELI_THR_IDLE"), &idle);

	float max_throw = 0.f;
	param_set(param_find("CA_MAX_SVO_THROW"), &max_throw);
}

// Helper to initialize the object and run getEffectivenessMatrix to set _first_sp1_servo_index.
static ActuatorEffectivenessHelicopterDual *createAndInit(
	ActuatorEffectivenessHelicopterDual::DualMode mode,
	ActuatorEffectiveness::Configuration &config)
{
	auto *dual = new ActuatorEffectivenessHelicopterDual(nullptr, mode);
	dual->getEffectivenessMatrix(config, EffectivenessUpdateReason::CONFIGURATION_UPDATE);
	return dual;
}

// ============================================================================
// Test 1: Tandem pure pitch -> differential collective
// ============================================================================
TEST(ActuatorEffectivenessHelicopterDualTest, TandemPurePitchDifferentialCollective)
{
	setupDualParams(0.5f, 1.f, 0);

	ActuatorEffectiveness::Configuration config{};
	auto *dual = createAndInit(ActuatorEffectivenessHelicopterDual::DualMode::TANDEM, config);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::PITCH) = 0.5f;
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -0.5f; // 50% thrust

	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp.setAll(0.f);
	ActuatorEffectiveness::ActuatorVector actuator_min{};
	actuator_min.setAll(-1.f);
	ActuatorEffectiveness::ActuatorVector actuator_max{};
	actuator_max.setAll(1.f);

	dual->updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);

	// With linear pitch curve at 50%: collective = interpolateN(0.5, {-0.5,-0.25,0,0.25,0.5}) = 0.0
	// Tandem: collective1 = collective + pitch * DCP = 0.0 + 0.5 * 0.5 = 0.25
	//         collective2 = collective - pitch * DCP = 0.0 - 0.5 * 0.5 = -0.25
	// pitch1 = yaw * yaw_scaler = 0, roll1 = roll = 0
	// SP1 servo 0 (0 deg): output = collective1 + 0*1.0 - 0*0.0 = 0.25
	// SP2 servo 0 (0 deg): output = collective2 + 0*1.0 - 0*0.0 = -0.25
	EXPECT_NEAR(actuator_sp(0), 0.25f, 1e-5f);  // SP1 servo 0
	EXPECT_NEAR(actuator_sp(3), -0.25f, 1e-5f); // SP2 servo 0

	// Verify all SP1 servos share the same collective offset (but with cyclic variation)
	const float angles_deg[] = {0.f, 140.f, 220.f};

	for (int i = 0; i < 3; ++i) {
		const float angle_rad = math::radians(angles_deg[i]);
		const float pitch_coeff = cosf(angle_rad);
		const float roll_coeff = sinf(angle_rad);
		const float expected1 = 0.25f + 0.f * pitch_coeff - 0.f * roll_coeff;
		const float expected2 = -0.25f + 0.f * pitch_coeff - 0.f * roll_coeff;
		EXPECT_NEAR(actuator_sp(i), expected1, 1e-5f);
		EXPECT_NEAR(actuator_sp(3 + i), expected2, 1e-5f);
	}

	delete dual;
}

// ============================================================================
// Test 2: Tandem pure yaw -> differential cyclic (pitch channel on swashplates)
// ============================================================================
TEST(ActuatorEffectivenessHelicopterDualTest, TandemPureYawDifferentialCyclic)
{
	setupDualParams(0.25f, 1.f, 0);

	ActuatorEffectiveness::Configuration config{};
	auto *dual = createAndInit(ActuatorEffectivenessHelicopterDual::DualMode::TANDEM, config);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 0.5f;
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -0.5f; // 50% thrust

	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp.setAll(0.f);
	ActuatorEffectiveness::ActuatorVector actuator_min{};
	actuator_min.setAll(-1.f);
	ActuatorEffectiveness::ActuatorVector actuator_max{};
	actuator_max.setAll(1.f);

	dual->updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);

	// collective = 0 at 50%, roll = 0, pitch = 0, yaw = 0.5
	// Tandem: pitch1 = yaw * yaw_scaler = 0.5, pitch2 = -0.5
	//         roll1 = roll2 = 0, collective1 = collective2 = 0
	// SP1 servo 0 (0 deg): output = 0 + 0.5 * cos(0) - 0*sin(0) = 0.5
	// SP2 servo 0 (0 deg): output = 0 + (-0.5) * cos(0) - 0*sin(0) = -0.5
	EXPECT_NEAR(actuator_sp(0), 0.5f, 1e-5f);
	EXPECT_NEAR(actuator_sp(3), -0.5f, 1e-5f);

	// Verify all servos with geometry
	const float angles_deg[] = {0.f, 140.f, 220.f};

	for (int i = 0; i < 3; ++i) {
		const float angle_rad = math::radians(angles_deg[i]);
		const float pitch_coeff = cosf(angle_rad);
		const float expected1 = 0.5f * pitch_coeff;  // collective=0, roll=0
		const float expected2 = -0.5f * pitch_coeff;
		EXPECT_NEAR(actuator_sp(i), expected1, 1e-5f);
		EXPECT_NEAR(actuator_sp(3 + i), expected2, 1e-5f);
	}

	delete dual;
}

// ============================================================================
// Test 3: Transverse pure roll -> differential collective
// ============================================================================
TEST(ActuatorEffectivenessHelicopterDualTest, TransversePureRollDifferentialCollective)
{
	setupDualParams(0.5f, 1.f, 0);

	ActuatorEffectiveness::Configuration config{};
	auto *dual = createAndInit(ActuatorEffectivenessHelicopterDual::DualMode::TRANSVERSE, config);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = 0.5f;
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -0.5f;

	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp.setAll(0.f);
	ActuatorEffectiveness::ActuatorVector actuator_min{};
	actuator_min.setAll(-1.f);
	ActuatorEffectiveness::ActuatorVector actuator_max{};
	actuator_max.setAll(1.f);

	dual->updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);

	// collective = 0 at 50%, pitch = 0, yaw = 0
	// Transverse: collective1 = 0 + 0.5 * 0.5 = 0.25, collective2 = 0 - 0.5 * 0.5 = -0.25
	//             pitch1 = pitch2 = 0, roll1 = roll2 = 0 (yaw is 0)
	// SP1 servo 0: output = 0.25
	// SP2 servo 0: output = -0.25
	EXPECT_NEAR(actuator_sp(0), 0.25f, 1e-5f);
	EXPECT_NEAR(actuator_sp(3), -0.25f, 1e-5f);

	delete dual;
}

// ============================================================================
// Test 4: Transverse pure yaw -> differential roll cyclic
// ============================================================================
TEST(ActuatorEffectivenessHelicopterDualTest, TransversePureYawDifferentialCyclic)
{
	setupDualParams(0.25f, 1.f, 0);

	ActuatorEffectiveness::Configuration config{};
	auto *dual = createAndInit(ActuatorEffectivenessHelicopterDual::DualMode::TRANSVERSE, config);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 0.5f;
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -0.5f;

	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp.setAll(0.f);
	ActuatorEffectiveness::ActuatorVector actuator_min{};
	actuator_min.setAll(-1.f);
	ActuatorEffectiveness::ActuatorVector actuator_max{};
	actuator_max.setAll(1.f);

	dual->updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);

	// collective = 0 at 50%, pitch = 0, roll = 0
	// Transverse: roll1 = yaw * yaw_scaler = 0.5, roll2 = -0.5
	//             pitch1 = pitch2 = 0, collective1 = collective2 = 0
	// SP1 servo 0 (0 deg): output = 0 + 0*cos(0) - 0.5*sin(0) = 0 (sin(0)=0)
	// SP1 servo 1 (140 deg): output = 0 + 0*cos(140) - 0.5*sin(140)
	// SP2 servo 1 (140 deg): output = 0 + 0*cos(140) - (-0.5)*sin(140) = +0.5*sin(140)
	const float angles_deg[] = {0.f, 140.f, 220.f};

	for (int i = 0; i < 3; ++i) {
		const float angle_rad = math::radians(angles_deg[i]);
		const float roll_coeff = sinf(angle_rad);
		const float expected1 = -0.5f * roll_coeff;   // -roll1 * roll_coeff
		const float expected2 = 0.5f * roll_coeff;    // -roll2 * roll_coeff = -(-0.5)*roll_coeff
		EXPECT_NEAR(actuator_sp(i), expected1, 1e-5f);
		EXPECT_NEAR(actuator_sp(3 + i), expected2, 1e-5f);
	}

	delete dual;
}

// ============================================================================
// Test 5: Single engine -> second throttle is NAN
// ============================================================================
TEST(ActuatorEffectivenessHelicopterDualTest, SingleEngineThrottle)
{
	setupDualParams(0.25f, 1.f, 0);

	ActuatorEffectiveness::Configuration config{};
	auto *dual = createAndInit(ActuatorEffectivenessHelicopterDual::DualMode::TANDEM, config);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -0.5f;

	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp.setAll(0.f);
	ActuatorEffectiveness::ActuatorVector actuator_min{};
	actuator_min.setAll(-1.f);
	ActuatorEffectiveness::ActuatorVector actuator_max{};
	actuator_max.setAll(1.f);

	dual->updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);

	// Layout: SP1(3 servos) + SP2(3 servos) + throttle1 + throttle2
	// Index 6 = throttle1, Index 7 = throttle2
	// Not armed -> spool state SHUT_DOWN -> throttle = 0
	EXPECT_FLOAT_EQ(actuator_sp(6), 0.f); // throttle1 = 0 (shut down)
	EXPECT_TRUE(std::isnan(actuator_sp(7))); // throttle2 = NAN (single engine)

	delete dual;
}

// ============================================================================
// Test 6: Dual engine -> both throttles are finite
// ============================================================================
TEST(ActuatorEffectivenessHelicopterDualTest, DualEngineThrottle)
{
	setupDualParams(0.25f, 1.f, 1);

	ActuatorEffectiveness::Configuration config{};
	auto *dual = createAndInit(ActuatorEffectivenessHelicopterDual::DualMode::TANDEM, config);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -0.5f;

	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp.setAll(0.f);
	ActuatorEffectiveness::ActuatorVector actuator_min{};
	actuator_min.setAll(-1.f);
	ActuatorEffectiveness::ActuatorVector actuator_max{};
	actuator_max.setAll(1.f);

	dual->updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);

	// Layout: SP1(3 servos) + SP2(3 servos) + throttle1 + throttle2
	// Not armed -> throttle = 0 for both, but both should be finite
	EXPECT_TRUE(std::isfinite(actuator_sp(6)));
	EXPECT_TRUE(std::isfinite(actuator_sp(7)));
	EXPECT_FLOAT_EQ(actuator_sp(6), actuator_sp(7)); // Both should be equal

	delete dual;
}
