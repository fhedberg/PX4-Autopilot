# Helicopter Dual-Rotor Support Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add tandem and transverse dual-rotor helicopter support to PX4 by extracting a reusable HeliSwashplate component, adding a formal spool state machine, and implementing differential collective pitch mixing.

**Architecture:** Composable swashplate components following PX4's existing ActuatorEffectiveness pattern. A new `HeliSwashplate` class encapsulates per-rotor-head mixing, used by both the refactored single-rotor class and the new dual-rotor class. CA_AIRFRAME values 16 (Dual Tandem) and 17 (Dual Transverse) are added.

**Tech Stack:** C++, PX4 uORB, PX4 parameter system, GTest, Gazebo SITL

**Working directory:** `/Users/fhedberg/src/ornen/PX4-Autopilot/.worktrees/feat-helicopter-dual-rotor`

**Important conventions:**
- Run `make format` on changed C/C++ files before committing
- Use conventional commit format: `type(scope): description`
- No Co-Authored-By or Claude attribution in commits
- Use the `/commit` skill for commits

---

### Task 1: Extract HeliSwashplate Component

Extract swashplate servo mixing logic from `ActuatorEffectivenessHelicopter` into a reusable `HeliSwashplate` class. The existing single-rotor behavior must remain identical.

**Files:**
- Create: `src/modules/control_allocator/VehicleActuatorEffectiveness/HeliSwashplate.hpp`
- Create: `src/modules/control_allocator/VehicleActuatorEffectiveness/HeliSwashplate.cpp`
- Modify: `src/modules/control_allocator/VehicleActuatorEffectiveness/CMakeLists.txt`

- [ ] **Step 1: Write HeliSwashplate unit test**

Create `src/modules/control_allocator/VehicleActuatorEffectiveness/HeliSwashplateTest.cpp`:

```cpp
#include <gtest/gtest.h>
#include "HeliSwashplate.hpp"

using namespace matrix;

TEST(HeliSwashplateTest, ThreeServoCollectiveOnly)
{
	HeliSwashplate swash;
	HeliSwashplate::Geometry geo{};
	geo.count = 3;
	// Standard 120-degree spacing: 0, 140, 220 degrees
	geo.servos[0] = {0.f, 1.f, 0.f};
	geo.servos[1] = {140.f, 1.f, 0.f};
	geo.servos[2] = {220.f, 1.f, 0.f};
	geo.max_servo_throw_deg = 0.f; // linearization disabled
	swash.setGeometry(geo);

	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	swash.mix(0.f, 0.f, 0.5f, actuator_sp, 0);

	// Pure collective: all servos should output 0.5
	EXPECT_NEAR(actuator_sp(0), 0.5f, 1e-5f);
	EXPECT_NEAR(actuator_sp(1), 0.5f, 1e-5f);
	EXPECT_NEAR(actuator_sp(2), 0.5f, 1e-5f);
}

TEST(HeliSwashplateTest, ThreeServoPurePitch)
{
	HeliSwashplate swash;
	HeliSwashplate::Geometry geo{};
	geo.count = 3;
	geo.servos[0] = {0.f, 1.f, 0.f};
	geo.servos[1] = {140.f, 1.f, 0.f};
	geo.servos[2] = {220.f, 1.f, 0.f};
	geo.max_servo_throw_deg = 0.f;
	swash.setGeometry(geo);

	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	swash.mix(0.f, 0.3f, 0.f, actuator_sp, 0);

	// Servo 0 at 0 deg: pitch_coeff = cos(0)*1 = 1.0, roll_coeff = sin(0)*1 = 0
	// output = 0 + 0.3*1.0 - 0*0 + 0 = 0.3
	EXPECT_NEAR(actuator_sp(0), 0.3f, 1e-5f);

	// Servo 1 at 140 deg: pitch_coeff = cos(140)*1 = -0.766, roll_coeff = sin(140)*1 = 0.643
	// output = 0 + 0.3*(-0.766) - 0*0.643 + 0 = -0.2298
	EXPECT_NEAR(actuator_sp(1), 0.3f * cosf(math::radians(140.f)), 1e-4f);

	// Servo 2 at 220 deg: pitch_coeff = cos(220)*1 = -0.766, roll_coeff = sin(220)*1 = -0.643
	// output = 0 + 0.3*(-0.766) - 0*(-0.643) + 0 = -0.2298
	EXPECT_NEAR(actuator_sp(2), 0.3f * cosf(math::radians(220.f)), 1e-4f);
}

TEST(HeliSwashplateTest, ThreeServoPureRoll)
{
	HeliSwashplate swash;
	HeliSwashplate::Geometry geo{};
	geo.count = 3;
	geo.servos[0] = {0.f, 1.f, 0.f};
	geo.servos[1] = {140.f, 1.f, 0.f};
	geo.servos[2] = {220.f, 1.f, 0.f};
	geo.max_servo_throw_deg = 0.f;
	swash.setGeometry(geo);

	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	swash.mix(0.4f, 0.f, 0.f, actuator_sp, 0);

	// Servo 0 at 0 deg: roll_coeff = sin(0)*1 = 0
	// output = 0 + 0 - 0.4*0 + 0 = 0
	EXPECT_NEAR(actuator_sp(0), 0.f, 1e-5f);

	// Servo 1 at 140 deg: roll_coeff = sin(140)*1 = 0.643
	// output = 0 + 0 - 0.4*0.643 + 0 = -0.257
	EXPECT_NEAR(actuator_sp(1), -0.4f * sinf(math::radians(140.f)), 1e-4f);

	// Servo 2 at 220 deg: roll_coeff = sin(220)*1 = -0.643
	// output = 0 + 0 - 0.4*(-0.643) + 0 = 0.257
	EXPECT_NEAR(actuator_sp(2), -0.4f * sinf(math::radians(220.f)), 1e-4f);
}

TEST(HeliSwashplateTest, FourServoMixing)
{
	HeliSwashplate swash;
	HeliSwashplate::Geometry geo{};
	geo.count = 4;
	// 90-degree spacing
	geo.servos[0] = {0.f, 1.f, 0.f};
	geo.servos[1] = {90.f, 1.f, 0.f};
	geo.servos[2] = {180.f, 1.f, 0.f};
	geo.servos[3] = {270.f, 1.f, 0.f};
	geo.max_servo_throw_deg = 0.f;
	swash.setGeometry(geo);

	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	swash.mix(0.f, 0.f, 0.5f, actuator_sp, 0);

	for (int i = 0; i < 4; i++) {
		EXPECT_NEAR(actuator_sp(i), 0.5f, 1e-5f);
	}
}

TEST(HeliSwashplateTest, TrimOffset)
{
	HeliSwashplate swash;
	HeliSwashplate::Geometry geo{};
	geo.count = 3;
	geo.servos[0] = {0.f, 1.f, 0.1f};
	geo.servos[1] = {140.f, 1.f, -0.05f};
	geo.servos[2] = {220.f, 1.f, 0.02f};
	geo.max_servo_throw_deg = 0.f;
	swash.setGeometry(geo);

	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	swash.mix(0.f, 0.f, 0.5f, actuator_sp, 0);

	EXPECT_NEAR(actuator_sp(0), 0.6f, 1e-5f);
	EXPECT_NEAR(actuator_sp(1), 0.45f, 1e-5f);
	EXPECT_NEAR(actuator_sp(2), 0.52f, 1e-5f);
}

TEST(HeliSwashplateTest, StartIndexOffset)
{
	HeliSwashplate swash;
	HeliSwashplate::Geometry geo{};
	geo.count = 3;
	geo.servos[0] = {0.f, 1.f, 0.f};
	geo.servos[1] = {140.f, 1.f, 0.f};
	geo.servos[2] = {220.f, 1.f, 0.f};
	geo.max_servo_throw_deg = 0.f;
	swash.setGeometry(geo);

	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp.setAll(0.f);
	swash.mix(0.f, 0.f, 0.5f, actuator_sp, 4);

	// Indices 0-3 should be untouched
	EXPECT_FLOAT_EQ(actuator_sp(0), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(3), 0.f);
	// Indices 4-6 should have the servo outputs
	EXPECT_NEAR(actuator_sp(4), 0.5f, 1e-5f);
	EXPECT_NEAR(actuator_sp(5), 0.5f, 1e-5f);
	EXPECT_NEAR(actuator_sp(6), 0.5f, 1e-5f);
}

TEST(HeliSwashplateTest, Linearization)
{
	HeliSwashplate swash;
	HeliSwashplate::Geometry geo{};
	geo.count = 3;
	geo.servos[0] = {0.f, 1.f, 0.f};
	geo.servos[1] = {120.f, 1.f, 0.f};
	geo.servos[2] = {240.f, 1.f, 0.f};
	geo.max_servo_throw_deg = 30.f; // enable linearization
	swash.setGeometry(geo);

	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	swash.mix(0.f, 0.f, 0.3f, actuator_sp, 0);

	// With linearization the output should differ from the linear case
	// Linearized: asin(sin(30deg) * 0.3) / 30deg = asin(0.15) / 0.5236 = 0.1506 / 0.5236 ~= 0.2876
	float expected = asinf(sinf(math::radians(30.f)) * 0.3f) / math::radians(30.f);
	EXPECT_NEAR(actuator_sp(0), expected, 1e-4f);
}
```

- [ ] **Step 2: Register test in CMakeLists.txt**

Add to `src/modules/control_allocator/VehicleActuatorEffectiveness/CMakeLists.txt`, after the existing gtest lines:

```cmake
px4_add_functional_gtest(SRC HeliSwashplateTest.cpp LINKLIBS VehicleActuatorEffectiveness)
```

- [ ] **Step 3: Write HeliSwashplate header**

Create `src/modules/control_allocator/VehicleActuatorEffectiveness/HeliSwashplate.hpp`:

```cpp
#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include <lib/mathlib/mathlib.h>

class HeliSwashplate {
public:
	static constexpr int NUM_SERVOS_MAX = 4;

	struct Geometry {
		struct Servo {
			float angle_deg;
			float arm_length;
			float trim;
		};
		Servo servos[NUM_SERVOS_MAX];
		int count{3};
		float max_servo_throw_deg{0.f};
	};

	void setGeometry(const Geometry &geometry);

	void mix(float roll, float pitch, float collective,
		 ActuatorEffectiveness::ActuatorVector &actuator_sp, int start_index) const;

	int count() const { return _count; }

private:
	struct ComputedServo {
		float pitch_coeff;
		float roll_coeff;
		float trim;
	};

	ComputedServo _servos[NUM_SERVOS_MAX]{};
	int _count{0};

	bool _linearize{false};
	float _max_servo_height{0.f};
	float _inverse_max_servo_throw{0.f};
};
```

- [ ] **Step 4: Write HeliSwashplate implementation**

Create `src/modules/control_allocator/VehicleActuatorEffectiveness/HeliSwashplate.cpp`:

```cpp
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
```

- [ ] **Step 5: Add source files to CMakeLists.txt**

Add `HeliSwashplate.hpp` and `HeliSwashplate.cpp` to the `px4_add_library(VehicleActuatorEffectiveness ...)` list in `src/modules/control_allocator/VehicleActuatorEffectiveness/CMakeLists.txt`.

- [ ] **Step 6: Build and run tests**

```bash
cd /Users/fhedberg/src/ornen/PX4-Autopilot/.worktrees/feat-helicopter-dual-rotor
make px4_sitl_default
# Then run the specific test:
build/px4_sitl_default/functional-HeliSwashplateTest
```

Expected: All 7 HeliSwashplate tests pass. The existing ActuatorEffectivenessHelicopterTest must also still pass.

- [ ] **Step 7: Commit**

Use `/commit` skill. Message: `feat(heli): add HeliSwashplate reusable component`

---

### Task 2: Refactor Single-Rotor to Use HeliSwashplate

Modify `ActuatorEffectivenessHelicopter` to delegate swashplate mixing to `HeliSwashplate`. All existing behavior and tests must remain identical.

**Files:**
- Modify: `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHelicopter.hpp`
- Modify: `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHelicopter.cpp`

- [ ] **Step 1: Add HeliSwashplate member to header**

In `ActuatorEffectivenessHelicopter.hpp`, add include and member:

```cpp
#include "HeliSwashplate.hpp"
```

Add private member:

```cpp
HeliSwashplate _swashplate;
```

- [ ] **Step 2: Populate swashplate geometry in updateParams()**

At the end of `ActuatorEffectivenessHelicopter::updateParams()`, after existing param loading, add:

```cpp
HeliSwashplate::Geometry swash_geo{};
swash_geo.count = _geometry.num_swash_plate_servos;

for (int i = 0; i < swash_geo.count; ++i) {
	swash_geo.servos[i].angle_deg = math::degrees(_geometry.swash_plate_servos[i].angle);
	swash_geo.servos[i].arm_length = _geometry.swash_plate_servos[i].arm_length;
	swash_geo.servos[i].trim = _geometry.swash_plate_servos[i].trim;
}

swash_geo.max_servo_throw_deg = max_servo_throw_deg;
_swashplate.setGeometry(swash_geo);
```

Note: `max_servo_throw_deg` is the local variable already computed in `updateParams()` — move it above this block if needed so it's in scope.

- [ ] **Step 3: Replace inline swashplate mixing in updateSetpoint()**

In `ActuatorEffectivenessHelicopter::updateSetpoint()`, replace lines 178-200 (the `for` loop over swash plate servos and saturation checks) with:

```cpp
_swashplate.mix(control_sp(ControlAxis::ROLL), control_sp(ControlAxis::PITCH),
		collective_pitch, actuator_sp, _first_swash_plate_servo_index);

// Saturation check for roll & pitch
for (int i = 0; i < _geometry.num_swash_plate_servos; i++) {
	if (actuator_sp(_first_swash_plate_servo_index + i) < actuator_min(_first_swash_plate_servo_index + i)) {
		float roll_coeff = sinf(_geometry.swash_plate_servos[i].angle) * _geometry.swash_plate_servos[i].arm_length;
		float pitch_coeff = cosf(_geometry.swash_plate_servos[i].angle) * _geometry.swash_plate_servos[i].arm_length;
		setSaturationFlag(roll_coeff, _saturation_flags.roll_pos, _saturation_flags.roll_neg);
		setSaturationFlag(pitch_coeff, _saturation_flags.pitch_neg, _saturation_flags.pitch_pos);

	} else if (actuator_sp(_first_swash_plate_servo_index + i) > actuator_max(_first_swash_plate_servo_index + i)) {
		float roll_coeff = sinf(_geometry.swash_plate_servos[i].angle) * _geometry.swash_plate_servos[i].arm_length;
		float pitch_coeff = cosf(_geometry.swash_plate_servos[i].angle) * _geometry.swash_plate_servos[i].arm_length;
		setSaturationFlag(roll_coeff, _saturation_flags.roll_neg, _saturation_flags.roll_pos);
		setSaturationFlag(pitch_coeff, _saturation_flags.pitch_pos, _saturation_flags.pitch_neg);
	}
}
```

This preserves identical behavior: `HeliSwashplate::mix()` now handles the servo output computation (including linearization), while the saturation check loop stays in the parent class since it needs access to `actuator_min`/`actuator_max` and the saturation flags.

- [ ] **Step 4: Remove getLinearServoOutput()**

Delete the `getLinearServoOutput()` method from the `.cpp` and its declaration from the `.hpp`. Linearization is now handled inside `HeliSwashplate::mix()`.

Also remove `linearize_servos`, `max_servo_height`, and `inverse_max_servo_throw` from the `Geometry` struct since they're no longer used by the parent class.

- [ ] **Step 5: Build and run existing tests**

```bash
cd /Users/fhedberg/src/ornen/PX4-Autopilot/.worktrees/feat-helicopter-dual-rotor
make px4_sitl_default
build/px4_sitl_default/functional-ActuatorEffectivenessHelicopterTest
build/px4_sitl_default/functional-HeliSwashplateTest
```

Expected: Both test suites pass. The throttle curve test values must be identical to before the refactor.

- [ ] **Step 6: Run format and commit**

```bash
make format
```

Use `/commit` skill. Message: `refactor(heli): delegate swashplate mixing to HeliSwashplate`

---

### Task 3: Add HelicopterStatus uORB Message

Add the `helicopter_status` uORB topic for publishing spool state.

**Files:**
- Create: `msg/HelicopterStatus.msg`
- Modify: `msg/CMakeLists.txt`

- [ ] **Step 1: Create message definition**

Create `msg/HelicopterStatus.msg`:

```
uint64 timestamp        # time since system start (microseconds)

# Spool state
uint8 SPOOL_STATE_SHUT_DOWN = 0
uint8 SPOOL_STATE_GROUND_IDLE = 1
uint8 SPOOL_STATE_SPOOLING_UP = 2
uint8 SPOOL_STATE_THROTTLE_UNLIMITED = 3
uint8 SPOOL_STATE_SPOOLING_DOWN = 4

uint8 spool_state       # current spool state
float32 throttle_output # current throttle output [0, 1]
float32 spoolup_progress # spoolup progress [0, 1]
```

- [ ] **Step 2: Register message in CMakeLists.txt**

Add `HelicopterStatus.msg` to `msg/CMakeLists.txt` in alphabetical order, after `HeaterStatus.msg`:

```cmake
	HelicopterStatus.msg
```

- [ ] **Step 3: Build to verify message generation**

```bash
cd /Users/fhedberg/src/ornen/PX4-Autopilot/.worktrees/feat-helicopter-dual-rotor
make px4_sitl_default
```

Expected: Build succeeds and generates `uORB/topics/helicopter_status.h`.

- [ ] **Step 4: Commit**

Use `/commit` skill. Message: `feat(heli): add helicopter_status uORB message`

---

### Task 4: Add Spool State Machine

Add formal spool state management and idle throttle parameter. Integrate with the existing single-rotor class and publish `helicopter_status`.

**Files:**
- Modify: `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHelicopter.hpp`
- Modify: `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHelicopter.cpp`
- Modify: `src/modules/control_allocator/module.yaml`

- [ ] **Step 1: Add spool state enum and members to header**

In `ActuatorEffectivenessHelicopter.hpp`, add includes:

```cpp
#include <uORB/Publication.hpp>
#include <uORB/topics/helicopter_status.h>
```

Add public enum:

```cpp
enum class SpoolState : uint8_t {
	SHUT_DOWN = 0,
	GROUND_IDLE,
	SPOOLING_UP,
	THROTTLE_UNLIMITED,
	SPOOLING_DOWN,
};
```

Add to the `Geometry` struct:

```cpp
float throttle_idle;
```

Add to `ParamHandles`:

```cpp
param_t throttle_idle;
```

Add private members:

```cpp
SpoolState _spool_state{SpoolState::SHUT_DOWN};
uORB::Publication<helicopter_status_s> _helicopter_status_pub{ORB_ID(helicopter_status)};
```

Replace the `throttleSpoolupProgress()` method declaration with:

```cpp
void updateSpoolState();
float spoolupThrottle(float commanded_throttle);
```

- [ ] **Step 2: Add CA_HELI_THR_IDLE parameter to module.yaml**

Add to `src/modules/control_allocator/module.yaml` in the helicopter section, after `CA_MAX_SVO_THROW`:

```yaml
        CA_HELI_THR_IDLE:
          description:
            short: Idle throttle output for helicopter
            long: |
              Throttle output when in ground idle state (armed but rotor not at flight speed).
          type: float
          decimal: 3
          increment: 0.01
          min: 0
          max: 0.3
          default: 0.05
```

- [ ] **Step 3: Load idle throttle param in updateParams()**

In `ActuatorEffectivenessHelicopter::updateParams()`, add param handle init in the constructor:

```cpp
_param_handles.throttle_idle = param_find("CA_HELI_THR_IDLE");
```

And in `updateParams()`:

```cpp
param_get(_param_handles.throttle_idle, &_geometry.throttle_idle);
```

- [ ] **Step 4: Implement spool state machine**

Replace `throttleSpoolupProgress()` in the `.cpp` with:

```cpp
void ActuatorEffectivenessHelicopter::updateSpoolState()
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

			if (time_since_disarm > _geometry.spoolup_time) {
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
		if (mainMotorEnaged()) {
			_spool_state = SpoolState::SPOOLING_UP;
		}

		break;

	case SpoolState::SPOOLING_UP:
		if (time_since_arming >= _geometry.spoolup_time) {
			_spool_state = SpoolState::THROTTLE_UNLIMITED;
		}

		break;

	case SpoolState::THROTTLE_UNLIMITED:
		break;

	case SpoolState::SPOOLING_DOWN:
		break;
	}
}

float ActuatorEffectivenessHelicopter::spoolupThrottle(float commanded_throttle)
{
	switch (_spool_state) {
	case SpoolState::SHUT_DOWN:
		return 0.f;

	case SpoolState::GROUND_IDLE:
		return _geometry.throttle_idle;

	case SpoolState::SPOOLING_UP: {
		const float time_since_arming = (hrt_absolute_time() - _armed_time) / 1e6f;
		const float progress = math::constrain(time_since_arming / _geometry.spoolup_time, 0.f, 1.f);
		return commanded_throttle * progress;
	}

	case SpoolState::THROTTLE_UNLIMITED:
		return commanded_throttle;

	case SpoolState::SPOOLING_DOWN:
		return _geometry.throttle_idle;
	}

	return 0.f;
}
```

- [ ] **Step 5: Update updateSetpoint() to use new spool state**

Replace the spoolup_progress lines at the top of `updateSetpoint()`:

```cpp
updateSpoolState();

float rpm_control_output = 0;
#if CONTROL_ALLOCATOR_RPM_CONTROL
_rpm_control.setSpoolupProgress(_spool_state == SpoolState::THROTTLE_UNLIMITED ? 1.f : 0.f);
rpm_control_output = _rpm_control.getActuatorCorrection();
#endif

const float commanded_throttle = math::interpolateN(-control_sp(ControlAxis::THRUST_Z), _geometry.throttle_curve)
				 + rpm_control_output;
const float throttle = spoolupThrottle(commanded_throttle);
const float collective_pitch = math::interpolateN(-control_sp(ControlAxis::THRUST_Z), _geometry.pitch_curve);
```

- [ ] **Step 6: Publish helicopter status**

At the end of `updateSetpoint()`, add:

```cpp
helicopter_status_s status{};
status.timestamp = hrt_absolute_time();
status.spool_state = static_cast<uint8_t>(_spool_state);
status.throttle_output = throttle;
status.spoolup_progress = (_spool_state == SpoolState::THROTTLE_UNLIMITED) ? 1.f :
			  (_spool_state == SpoolState::SPOOLING_UP) ?
			  math::constrain((hrt_absolute_time() - _armed_time) / 1e6f / _geometry.spoolup_time, 0.f, 1.f) :
			  0.f;
_helicopter_status_pub.publish(status);
```

- [ ] **Step 7: Build and run tests**

```bash
cd /Users/fhedberg/src/ornen/PX4-Autopilot/.worktrees/feat-helicopter-dual-rotor
make px4_sitl_default
build/px4_sitl_default/functional-ActuatorEffectivenessHelicopterTest
```

Expected: existing throttle curve test still passes (the test doesn't arm, so spool state stays at SHUT_DOWN and throttle output is 0 — which matches the existing behavior where `spoolup_progress` starts at 0 for unarmed).

- [ ] **Step 8: Run format and commit**

```bash
make format
```

Use `/commit` skill. Message: `feat(heli): add spool state machine and helicopter_status topic`

---

### Task 5: Add CA_AIRFRAME Values and Dual Parameter Definitions

Add the enum values, parameter definitions, and ControlAllocator wiring for dual helicopter configurations.

**Files:**
- Modify: `src/modules/control_allocator/module.yaml`
- Modify: `src/modules/control_allocator/ControlAllocator.hpp`
- Modify: `src/modules/control_allocator/ControlAllocator.cpp`

- [ ] **Step 1: Add CA_AIRFRAME enum values to module.yaml**

In `src/modules/control_allocator/module.yaml`, add after value `15: Spacecraft 3D`:

```yaml
                16: Helicopter (Dual Tandem)
                17: Helicopter (Dual Transverse)
```

- [ ] **Step 2: Add dual helicopter parameters to module.yaml**

In the helicopter parameter section (after `CA_HELI_THR_IDLE`), add:

```yaml
        CA_SP1_COUNT:
            description:
                short: Number of swash plate servos for second rotor
            type: enum
            values:
                2: '2'
                3: '3'
                4: '4'
            default: 3
        CA_SP1_ANG${i}:
            description:
                short: Angle for second rotor swash plate servo ${i}
                long: |
                  The angle is measured clockwise (as seen from top), with 0 pointing forwards (X axis).
            type: float
            decimal: 0
            increment: 10
            unit: deg
            num_instances: 4
            min: 0
            max: 360
            default: [0, 140, 220, 0]
        CA_SP1_ARM_L${i}:
            description:
                short: Arm length for second rotor swash plate servo ${i}
                long: |
                  This is relative to the other arm lengths.
            type: float
            decimal: 3
            increment: 0.1
            num_instances: 4
            min: 0
            max: 10
            default: 1.0
        CA_HELI_DCP_SC:
          description:
            short: Differential collective pitch scaler
            long: |
              Controls how aggressively differential collective pitch is used for pitch (tandem) or roll (transverse) control.
              Higher values give stronger pitch/roll authority from differential collective but may cause asymmetric thrust.
          type: float
          decimal: 3
          increment: 0.05
          min: 0
          max: 1
          default: 0.25
        CA_HELI_YAW_SC:
          description:
            short: Yaw scaler for dual helicopter
            long: |
              Controls the yaw authority via differential cyclic between the two rotors.
          type: float
          decimal: 3
          increment: 0.1
          min: 0
          max: 2
          default: 1.0
        CA_HELI_DUAL_ENG:
          description:
            short: Dual helicopter engine configuration
            long: |
              0: single engine driving both rotors (one throttle output).
              1: independent engines (two throttle outputs, required for tiltrotor composability).
          type: enum
          values:
            0: Single engine
            1: Independent engines
          default: 0
```

- [ ] **Step 3: Add enum values to ControlAllocator.hpp**

In `ControlAllocator.hpp`, add to the `EffectivenessSource` enum before the closing brace:

```cpp
		SPACECRAFT_3D = 15,
		HELICOPTER_DUAL_TANDEM = 16,
		HELICOPTER_DUAL_TRANSVERSE = 17,
```

Note: also add `SPACECRAFT_3D = 15` which exists in the YAML but was missing from the enum.

- [ ] **Step 4: Add switch cases to ControlAllocator.cpp**

In `ControlAllocator.cpp`, add `#include` at the top:

```cpp
#include "VehicleActuatorEffectiveness/ActuatorEffectivenessHelicopterDual.hpp"
```

In the `switch (source)` block in `update_effectiveness_source()`, add before the `default:` case:

```cpp
case EffectivenessSource::HELICOPTER_DUAL_TANDEM:
	tmp = new ActuatorEffectivenessHelicopterDual(this, ActuatorEffectivenessHelicopterDual::DualMode::TANDEM);
	break;

case EffectivenessSource::HELICOPTER_DUAL_TRANSVERSE:
	tmp = new ActuatorEffectivenessHelicopterDual(this, ActuatorEffectivenessHelicopterDual::DualMode::TRANSVERSE);
	break;
```

- [ ] **Step 5: Commit (won't build yet — dual class not created)**

Use `/commit` skill. Message: `feat(heli): add CA_AIRFRAME values and parameters for dual helicopter`

---

### Task 6: Implement ActuatorEffectivenessHelicopterDual

The core dual-rotor helicopter effectiveness class with tandem and transverse mixing.

**Files:**
- Create: `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHelicopterDual.hpp`
- Create: `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHelicopterDual.cpp`
- Modify: `src/modules/control_allocator/VehicleActuatorEffectiveness/CMakeLists.txt`

- [ ] **Step 1: Write dual helicopter unit tests**

Create `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHelicopterDualTest.cpp`:

```cpp
#include <gtest/gtest.h>
#include "ActuatorEffectivenessHelicopterDual.hpp"

using namespace matrix;

class HelicopterDualTest : public ::testing::Test {
protected:
	void SetUp() override
	{
		param_control_autosave(false);
	}

	void setDefaultParams()
	{
		// Swashplate 1
		int32_t count = 3;
		param_set(param_find("CA_SP0_COUNT"), &count);
		float ang0 = 0.f, ang1 = 140.f, ang2 = 220.f;
		param_set(param_find("CA_SP0_ANG0"), &ang0);
		param_set(param_find("CA_SP0_ANG1"), &ang1);
		param_set(param_find("CA_SP0_ANG2"), &ang2);

		// Swashplate 2
		param_set(param_find("CA_SP1_COUNT"), &count);
		param_set(param_find("CA_SP1_ANG0"), &ang0);
		param_set(param_find("CA_SP1_ANG1"), &ang1);
		param_set(param_find("CA_SP1_ANG2"), &ang2);

		// DCP and yaw scalers
		float dcp = 0.5f;
		param_set(param_find("CA_HELI_DCP_SC"), &dcp);
		float yaw_sc = 1.0f;
		param_set(param_find("CA_HELI_YAW_SC"), &yaw_sc);

		// Throttle curve: linear
		for (int i = 0; i < 5; i++) {
			char buf[17];
			snprintf(buf, sizeof(buf), "CA_HELI_THR_C%u", i);
			float val = (float)i / 4.f;
			param_set(param_find(buf), &val);
		}

		// Pitch curve: linear -0.5 to 0.5
		for (int i = 0; i < 5; i++) {
			char buf[17];
			snprintf(buf, sizeof(buf), "CA_HELI_PITCH_C%u", i);
			float val = -0.5f + (float)i / 4.f;
			param_set(param_find(buf), &val);
		}
	}

	ActuatorEffectiveness::ActuatorVector runMixing(
		ActuatorEffectivenessHelicopterDual &dual,
		float roll, float pitch, float yaw, float thrust_z)
	{
		ActuatorEffectiveness::Configuration config{};
		dual.getEffectivenessMatrix(config, EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE);

		Vector<float, 6> control_sp{};
		control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = roll;
		control_sp(ActuatorEffectiveness::ControlAxis::PITCH) = pitch;
		control_sp(ActuatorEffectiveness::ControlAxis::YAW) = yaw;
		control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = thrust_z;

		ActuatorEffectiveness::ActuatorVector actuator_sp{};
		ActuatorEffectiveness::ActuatorVector actuator_min{};
		actuator_min.setAll(-1.f);
		ActuatorEffectiveness::ActuatorVector actuator_max{};
		actuator_max.setAll(1.f);

		dual.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
		return actuator_sp;
	}
};

TEST_F(HelicopterDualTest, TandemPurePitchDifferentialCollective)
{
	setDefaultParams();
	ActuatorEffectivenessHelicopterDual dual(nullptr,
			ActuatorEffectivenessHelicopterDual::DualMode::TANDEM);

	// Pure pitch input with 50% thrust
	auto sp = runMixing(dual, 0.f, 0.5f, 0.f, -0.5f);

	// At 50% thrust: collective = 0.0 (midpoint of pitch curve -0.5 to 0.5)
	// DCP_SC = 0.5, pitch = 0.5
	// coll1 = 0.0 + 0.5 * 0.5 = 0.25
	// coll2 = 0.0 - 0.5 * 0.5 = -0.25
	// Swashplate 1 servo 0 (0 deg): output = coll1 = 0.25
	// Swashplate 2 servo 0 (0 deg): output = coll2 = -0.25
	EXPECT_NEAR(sp(0), 0.25f, 1e-4f);  // SP1 servo 0
	EXPECT_NEAR(sp(3), -0.25f, 1e-4f); // SP2 servo 0
}

TEST_F(HelicopterDualTest, TandemPureYawDifferentialCyclic)
{
	setDefaultParams();
	ActuatorEffectivenessHelicopterDual dual(nullptr,
			ActuatorEffectivenessHelicopterDual::DualMode::TANDEM);

	// Pure yaw input, no thrust
	auto sp = runMixing(dual, 0.f, 0.f, 0.5f, -0.5f);

	// Tandem yaw: pitch1 = yaw * yaw_scaler = 0.5, pitch2 = -0.5
	// SP1 servo 0 (0 deg): pitch_coeff = cos(0) = 1
	// output = coll + 0.5 * 1 = 0 + 0.5 = 0.5
	// SP2 servo 0 (0 deg): output = coll + (-0.5) * 1 = -0.5
	EXPECT_NEAR(sp(0), 0.5f, 1e-4f);   // SP1 servo 0: positive cyclic
	EXPECT_NEAR(sp(3), -0.5f, 1e-4f);  // SP2 servo 0: negative cyclic
}

TEST_F(HelicopterDualTest, TransversePureRollDifferentialCollective)
{
	setDefaultParams();
	ActuatorEffectivenessHelicopterDual dual(nullptr,
			ActuatorEffectivenessHelicopterDual::DualMode::TRANSVERSE);

	// Pure roll input with 50% thrust
	auto sp = runMixing(dual, 0.5f, 0.f, 0.f, -0.5f);

	// At 50% thrust: collective = 0.0
	// DCP_SC = 0.5, roll = 0.5
	// coll1 = 0.0 + 0.5 * 0.5 = 0.25
	// coll2 = 0.0 - 0.5 * 0.5 = -0.25
	EXPECT_NEAR(sp(0), 0.25f, 1e-4f);  // SP1 servo 0
	EXPECT_NEAR(sp(3), -0.25f, 1e-4f); // SP2 servo 0
}

TEST_F(HelicopterDualTest, TransversePureYawDifferentialCyclic)
{
	setDefaultParams();
	ActuatorEffectivenessHelicopterDual dual(nullptr,
			ActuatorEffectivenessHelicopterDual::DualMode::TRANSVERSE);

	// Pure yaw input, no thrust
	auto sp = runMixing(dual, 0.f, 0.f, 0.5f, -0.5f);

	// Transverse yaw: roll1 = yaw * yaw_scaler = 0.5, roll2 = -0.5
	// SP1 servo 0 (0 deg): roll_coeff = sin(0) = 0
	// output = coll - 0.5 * 0 = 0
	EXPECT_NEAR(sp(0), 0.f, 1e-4f);

	// SP1 servo 1 (140 deg): roll_coeff = sin(140) = 0.643
	// output = 0 - 0.5 * 0.643 = -0.321
	EXPECT_NEAR(sp(1), -0.5f * sinf(math::radians(140.f)), 1e-3f);

	// SP2 servo 1: roll2 = -0.5
	// output = 0 - (-0.5) * 0.643 = 0.321
	EXPECT_NEAR(sp(4), 0.5f * sinf(math::radians(140.f)), 1e-3f);
}

TEST_F(HelicopterDualTest, SingleEngineThrottle)
{
	setDefaultParams();
	int32_t single = 0;
	param_set(param_find("CA_HELI_DUAL_ENG"), &single);

	ActuatorEffectivenessHelicopterDual dual(nullptr,
			ActuatorEffectivenessHelicopterDual::DualMode::TANDEM);

	auto sp = runMixing(dual, 0.f, 0.f, 0.f, -0.5f);

	// Throttle at index 6 (after 2x3 servos), throttle 2 at index 7 should be NAN
	// Note: throttle will be 0 because we're not armed (spoolup = 0)
	EXPECT_FALSE(PX4_ISFINITE(sp(7))); // second throttle should be NAN for single engine
}

TEST_F(HelicopterDualTest, DualEngineThrottle)
{
	setDefaultParams();
	int32_t dual_eng = 1;
	param_set(param_find("CA_HELI_DUAL_ENG"), &dual_eng);

	ActuatorEffectivenessHelicopterDual dual(nullptr,
			ActuatorEffectivenessHelicopterDual::DualMode::TANDEM);

	auto sp = runMixing(dual, 0.f, 0.f, 0.f, -0.5f);

	// Both throttle outputs should be finite (even if 0 due to no arming)
	// Index 6 = throttle 1, index 7 = throttle 2
	// Both will be 0 (not armed), but both should be set (not NAN)
	EXPECT_TRUE(PX4_ISFINITE(sp(6)));
	EXPECT_TRUE(PX4_ISFINITE(sp(7)));
}
```

- [ ] **Step 2: Register dual test in CMakeLists.txt**

Add to `src/modules/control_allocator/VehicleActuatorEffectiveness/CMakeLists.txt`:

```cmake
px4_add_functional_gtest(SRC ActuatorEffectivenessHelicopterDualTest.cpp LINKLIBS VehicleActuatorEffectiveness)
```

- [ ] **Step 3: Write dual helicopter header**

Create `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHelicopterDual.hpp`:

```cpp
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

	// Spool state
	SpoolState _spool_state{SpoolState::SHUT_DOWN};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Publication<helicopter_status_s> _helicopter_status_pub{ORB_ID(helicopter_status)};
	bool _armed{false};
	uint64_t _armed_time{0};
};
```

- [ ] **Step 4: Write dual helicopter implementation**

Create `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHelicopterDual.cpp`:

```cpp
#include "ActuatorEffectivenessHelicopterDual.hpp"
#include <lib/mathlib/mathlib.h>

using namespace matrix;

ActuatorEffectivenessHelicopterDual::ActuatorEffectivenessHelicopterDual(ModuleParams *parent, DualMode mode)
	: ModuleParams(parent), _mode(mode)
{
	for (int sp = 0; sp < 2; ++sp) {
		const char *prefix = (sp == 0) ? "CA_SP0" : "CA_SP1";
		char buf[17];

		for (int i = 0; i < NUM_SWASH_PLATE_SERVOS_MAX; ++i) {
			snprintf(buf, sizeof(buf), "%s_ANG%u", prefix, i);
			_param_handles.sp[sp].angle[i] = param_find(buf);
			snprintf(buf, sizeof(buf), "%s_ARM_L%u", prefix, i);
			_param_handles.sp[sp].arm_length[i] = param_find(buf);
		}

		snprintf(buf, sizeof(buf), "%s_COUNT", prefix);
		_param_handles.sp[sp].count = param_find(buf);
	}

	for (int i = 0; i < 2 * NUM_SWASH_PLATE_SERVOS_MAX; ++i) {
		char buf[17];
		snprintf(buf, sizeof(buf), "CA_SV_CS%u_TRIM", i);
		_param_handles.trim[i] = param_find(buf);
	}

	for (int i = 0; i < NUM_CURVE_POINTS; ++i) {
		char buf[17];
		snprintf(buf, sizeof(buf), "CA_HELI_THR_C%u", i);
		_param_handles.throttle_curve[i] = param_find(buf);
		snprintf(buf, sizeof(buf), "CA_HELI_PITCH_C%u", i);
		_param_handles.pitch_curve[i] = param_find(buf);
	}

	_param_handles.dcp_scaler = param_find("CA_HELI_DCP_SC");
	_param_handles.yaw_scaler = param_find("CA_HELI_YAW_SC");
	_param_handles.dual_engine = param_find("CA_HELI_DUAL_ENG");
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

	for (int sp = 0; sp < 2; ++sp) {
		HeliSwashplate::Geometry geo{};

		int32_t count = 3;
		param_get(_param_handles.sp[sp].count, &count);
		geo.count = math::constrain((int)count, 2, NUM_SWASH_PLATE_SERVOS_MAX);

		for (int i = 0; i < geo.count; ++i) {
			param_get(_param_handles.sp[sp].angle[i], &geo.servos[i].angle_deg);
			param_get(_param_handles.sp[sp].arm_length[i], &geo.servos[i].arm_length);

			int trim_idx = sp * NUM_SWASH_PLATE_SERVOS_MAX + i;
			param_get(_param_handles.trim[trim_idx], &geo.servos[i].trim);
		}

		geo.max_servo_throw_deg = max_servo_throw_deg;

		if (sp == 0) {
			_swashplate1.setGeometry(geo);
			_sp1_count = geo.count;

		} else {
			_swashplate2.setGeometry(geo);
			_sp2_count = geo.count;
		}
	}

	for (int i = 0; i < NUM_CURVE_POINTS; ++i) {
		param_get(_param_handles.throttle_curve[i], &_throttle_curve[i]);
		param_get(_param_handles.pitch_curve[i], &_pitch_curve[i]);
	}

	param_get(_param_handles.dcp_scaler, &_dcp_scaler);
	param_get(_param_handles.yaw_scaler, &_yaw_scaler);
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

	_first_sp1_servo_index = configuration.num_actuators_matrix[0];

	// Swashplate 1 servos
	for (int i = 0; i < _sp1_count; ++i) {
		configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
	}

	// Swashplate 2 servos
	for (int i = 0; i < _sp2_count; ++i) {
		configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
	}

	// Throttle 1
	configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{});

	// Throttle 2 (if independent engines)
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

	const float commanded_throttle = math::interpolateN(-control_sp(ControlAxis::THRUST_Z), _throttle_curve);
	const float throttle = spoolupThrottle(commanded_throttle);
	const float collective = math::interpolateN(-control_sp(ControlAxis::THRUST_Z), _pitch_curve);

	const float roll = control_sp(ControlAxis::ROLL);
	const float pitch = control_sp(ControlAxis::PITCH);
	const float yaw = control_sp(ControlAxis::YAW);

	float roll1, pitch1, collective1;
	float roll2, pitch2, collective2;

	if (_mode == DualMode::TANDEM) {
		roll1 = roll;
		roll2 = roll;
		pitch1 = yaw * _yaw_scaler;
		pitch2 = -yaw * _yaw_scaler;
		collective1 = collective + pitch * _dcp_scaler;
		collective2 = collective - pitch * _dcp_scaler;

	} else { // TRANSVERSE
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

	// Throttle outputs
	const int throttle1_idx = sp2_start + _sp2_count;
	actuator_sp(throttle1_idx) = throttle;

	if (_dual_engine) {
		actuator_sp(throttle1_idx + 1) = throttle;

	} else {
		actuator_sp(throttle1_idx + 1) = NAN;
	}

	// Publish helicopter status
	helicopter_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.spool_state = static_cast<uint8_t>(_spool_state);
	status.throttle_output = throttle;
	status.spoolup_progress = (_spool_state == SpoolState::THROTTLE_UNLIMITED) ? 1.f :
				  (_spool_state == SpoolState::SPOOLING_UP) ?
				  math::constrain((hrt_absolute_time() - _armed_time) / 1e6f / _spoolup_time, 0.f, 1.f) :
				  0.f;
	_helicopter_status_pub.publish(status);
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

void ActuatorEffectivenessHelicopterDual::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
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
```

- [ ] **Step 5: Add source files to CMakeLists.txt**

Add `ActuatorEffectivenessHelicopterDual.hpp` and `ActuatorEffectivenessHelicopterDual.cpp` to the `px4_add_library(VehicleActuatorEffectiveness ...)` list.

- [ ] **Step 6: Build and run all tests**

```bash
cd /Users/fhedberg/src/ornen/PX4-Autopilot/.worktrees/feat-helicopter-dual-rotor
make px4_sitl_default
build/px4_sitl_default/functional-ActuatorEffectivenessHelicopterDualTest
build/px4_sitl_default/functional-HeliSwashplateTest
build/px4_sitl_default/functional-ActuatorEffectivenessHelicopterTest
```

Expected: All test suites pass.

- [ ] **Step 7: Run format and commit**

```bash
make format
```

Use `/commit` skill. Message: `feat(heli): implement dual helicopter tandem and transverse mixing`

---

### Task 7: Add ROMFS Airframe Configurations

Create airframe definitions for dual tandem and transverse helicopters.

**Files:**
- Create: `ROMFS/px4fmu_common/init.d/airframes/16002_helicopter_dual_tandem`
- Create: `ROMFS/px4fmu_common/init.d/airframes/16003_helicopter_dual_transverse`
- Modify: `ROMFS/px4fmu_common/init.d/rc.heli_defaults` (if needed)

- [ ] **Step 1: Create tandem airframe config**

Create `ROMFS/px4fmu_common/init.d/airframes/16002_helicopter_dual_tandem`:

```sh
#!/bin/sh
#
# @name Generic Helicopter (Dual Tandem)
#
# @type Helicopter
# @class Copter
#
# @board px4_fmu-v2 exclude
# @board bitcraze_crazyflie exclude
#

. ${R}etc/init.d/rc.heli_defaults

param set-default MC_ROLLRATE_P 0
param set-default MC_ROLLRATE_I 0
param set-default MC_ROLLRATE_D 0
param set-default MC_ROLLRATE_FF 0.1
param set-default MC_PITCHRATE_P 0
param set-default MC_PITCHRATE_I 0
param set-default MC_PITCHRATE_D 0
param set-default MC_PITCHRATE_FF 0.1
param set-default MC_YAWRATE_P 0
param set-default MC_YAWRATE_I 0
param set-default MC_YAWRATE_D 0
param set-default MC_YAWRATE_FF 0.1

param set-default CA_AIRFRAME 16
param set-default CA_HELI_DCP_SC 0.25
param set-default CA_HELI_YAW_SC 1.0
param set-default CA_HELI_DUAL_ENG 0
```

- [ ] **Step 2: Create transverse airframe config**

Create `ROMFS/px4fmu_common/init.d/airframes/16003_helicopter_dual_transverse`:

```sh
#!/bin/sh
#
# @name Generic Helicopter (Dual Transverse)
#
# @type Helicopter
# @class Copter
#
# @board px4_fmu-v2 exclude
# @board bitcraze_crazyflie exclude
#

. ${R}etc/init.d/rc.heli_defaults

param set-default MC_ROLLRATE_P 0
param set-default MC_ROLLRATE_I 0
param set-default MC_ROLLRATE_D 0
param set-default MC_ROLLRATE_FF 0.1
param set-default MC_PITCHRATE_P 0
param set-default MC_PITCHRATE_I 0
param set-default MC_PITCHRATE_D 0
param set-default MC_PITCHRATE_FF 0.1
param set-default MC_YAWRATE_P 0
param set-default MC_YAWRATE_I 0
param set-default MC_YAWRATE_D 0
param set-default MC_YAWRATE_FF 0.1

param set-default CA_AIRFRAME 17
param set-default CA_HELI_DCP_SC 0.25
param set-default CA_HELI_YAW_SC 1.0
param set-default CA_HELI_DUAL_ENG 1
```

- [ ] **Step 3: Build to verify airframe registration**

```bash
cd /Users/fhedberg/src/ornen/PX4-Autopilot/.worktrees/feat-helicopter-dual-rotor
make px4_sitl_default
```

Expected: Build succeeds and new airframes appear in the generated airframe list.

- [ ] **Step 4: Commit**

Use `/commit` skill. Message: `feat(heli): add dual tandem and transverse airframe configs`

---

### Task 8: Integration Build and Full Test Run

Final verification that everything builds and all tests pass together.

**Files:** None (verification only)

- [ ] **Step 1: Clean build**

```bash
cd /Users/fhedberg/src/ornen/PX4-Autopilot/.worktrees/feat-helicopter-dual-rotor
make clean
make px4_sitl_default
```

Expected: Full build succeeds with no warnings in our new files.

- [ ] **Step 2: Run all helicopter tests**

```bash
build/px4_sitl_default/functional-HeliSwashplateTest
build/px4_sitl_default/functional-ActuatorEffectivenessHelicopterTest
build/px4_sitl_default/functional-ActuatorEffectivenessHelicopterDualTest
```

Expected: All tests pass.

- [ ] **Step 3: Run full test suite**

```bash
make tests TESTFILTER="ActuatorEffectiveness"
```

Expected: All ActuatorEffectiveness tests pass (including existing rotors test).

- [ ] **Step 4: Verify format compliance**

```bash
make check_format
```

Expected: No formatting violations in new or modified files.

---

### Deferred: Gazebo SITL Models

Creating Gazebo SDF models for tandem and transverse dual helicopters requires 3D modeling (rotor geometry, inertia, visual meshes) and Gazebo-specific plugin configuration. This is a separate effort that should be done after the firmware is validated with unit tests. The firmware work in Tasks 1-8 is fully testable via unit tests without SITL models.
