# PX4 Helicopter Dual-Rotor Support Design

## Goal

Make PX4 a viable alternative to ArduPilot for traditional helicopters by adding
dual-rotor helicopter support (tandem and transverse configurations), improving
the single-rotor spool state machine, and architecting for future V-22
Osprey-style dual-heli tiltrotor composability.

## Scope

### In scope

- **Dual helicopter support**: tandem (front/aft, CH-47) and transverse
  (side-by-side, V-22 heli mode) configurations with differential collective
  pitch mixing
- **HeliSwashplate component**: reusable swashplate mixing extracted from
  existing single-rotor code, shared by single and dual
- **Spool state machine**: formal state machine (SHUT_DOWN, GROUND_IDLE,
  SPOOLING_UP, THROTTLE_UNLIMITED, SPOOLING_DOWN) with uORB status publishing
- **Refactored single-rotor**: existing `ActuatorEffectivenessHelicopter` refactored
  to use `HeliSwashplate`, preserving all current behavior
- **V-22 composability**: architecture designed so dual-heli code composes with
  existing tiltrotor VTOL framework (not implemented this phase)
- **Unit tests**: thorough tests for swashplate mixing, dual mixing, and spool states
- **Gazebo SITL models**: basic tandem and transverse models for flight testing
- **Airframe configs**: ROMFS airframe definitions for dual tandem and transverse

### Out of scope

- Governor/RSC (ESC handles rotor speed control)
- Helicopter-specific attitude/rate controller (leaky integrator, hover trim)
- Named swashplate presets (H1, H3, etc.) -- generic angle-based system is sufficient
- Additional tail rotor types (current ESC/servo is sufficient)
- Helicopter-specific flight modes (Stabilize/Acro with collective scaling)
- Autorotation
- Intermeshing dual mode
- Quad collective-pitch helicopter
- V-22 tiltrotor implementation (architecture only)

## Architecture

### Approach: composable swashplate components

Follow PX4's established pattern for complex vehicle types (as used by tiltrotor
VTOL): compose effectiveness classes from reusable components, use nonlinear
`updateSetpoint()` for helicopter-specific mixing, and communicate via custom
uORB topics where needed.

### File layout

```
src/modules/control_allocator/VehicleActuatorEffectiveness/
  HeliSwashplate.hpp/cpp                         # NEW: reusable component
  ActuatorEffectivenessHelicopter.hpp/cpp         # MODIFIED: uses HeliSwashplate
  ActuatorEffectivenessHelicopterCoaxial.hpp/cpp   # EXISTING (unchanged)
  ActuatorEffectivenessHelicopterDual.hpp/cpp      # NEW: tandem/transverse
  RpmControl.hpp/cpp                               # EXISTING (unchanged)
  ActuatorEffectivenessHelicopterTest.cpp           # MODIFIED: add dual tests

ROMFS/px4fmu_common/init.d/airframes/
  16001_helicopter                                # EXISTING
  16002_helicopter_dual_tandem                    # NEW
  16003_helicopter_dual_transverse                # NEW
```

### CA_AIRFRAME enum extensions

| Value | Configuration |
|-------|--------------|
| 10 | Helicopter (Tail ESC) -- existing |
| 11 | Helicopter (Tail Servo) -- existing |
| 12 | Helicopter (Coaxial) -- existing |
| 13 | Helicopter (Dual Tandem) -- NEW |
| 14 | Helicopter (Dual Transverse) -- NEW |

## Component Design

### HeliSwashplate

Reusable component encapsulating swashplate servo mixing for one rotor head.
Extracted from existing `ActuatorEffectivenessHelicopter::updateSetpoint()`.

```cpp
class HeliSwashplate {
public:
    struct Geometry {
        struct Servo {
            float angle;       // degrees, clockwise from forward
            float arm_length;  // relative effectiveness
            float trim;        // offset
        };
        Servo servos[4];
        int count;             // 2-4 servos
        float max_servo_throw; // linearization threshold (0 = disabled)
    };

    void setGeometry(const Geometry &geometry);

    void mix(float roll, float pitch, float collective,
             ActuatorVector &actuator_sp, int start_index) const;

    void getSaturationFlags(const ActuatorVector &actuator_sp, int start_index,
                            bool &roll_saturated, bool &pitch_saturated) const;
};
```

**Mixing math:**

```
For each servo i:
    pitch_coeff = cos(angle_i) * arm_length_i
    roll_coeff  = sin(angle_i) * arm_length_i
    output = collective + pitch * pitch_coeff - roll * roll_coeff + trim_i

    if max_servo_throw > 0:
        output = asin(output) * (max_throw / sin(max_throw))
```

The component does not own parameters. It receives `Geometry` from the parent
effectiveness class, which declares and manages parameters. This keeps the
component decoupled from the parameter namespace.

### ActuatorEffectivenessHelicopterDual

New effectiveness class for tandem and transverse dual-rotor helicopters.

**Dual mixing modes:**

| Axis | Tandem (front/aft) | Transverse (side-by-side) |
|------|-------------------|--------------------------|
| Roll | Both: cyclic roll | Differential collective |
| Pitch | Differential collective | Both: cyclic pitch |
| Yaw | Differential lateral cyclic | Differential longitudinal cyclic |
| Thrust | Both: collective (equal) | Both: collective (equal) |

**Core mixing logic:**

```cpp
if (mode == TANDEM) {
    roll1 = roll;               roll2 = roll;
    pitch1 = yaw * yaw_scaler;  pitch2 = -yaw * yaw_scaler;
    coll1 = coll + pitch * dcp; coll2 = coll - pitch * dcp;
} else { // TRANSVERSE
    pitch1 = pitch;             pitch2 = pitch;
    roll1 = yaw * yaw_scaler;   roll2 = -yaw * yaw_scaler;
    coll1 = coll + roll * dcp;  coll2 = coll - roll * dcp;
}

_swashplate1.mix(roll1, pitch1, coll1, actuator_sp, 0);
_swashplate2.mix(roll2, pitch2, coll2, actuator_sp, sp1_count);
actuator_sp(throttle_idx) = throttle * spoolup_progress;
```

**Engine configuration:** `CA_HELI_DUAL_ENG` parameter selects single engine
(one throttle output) or independent engines (two throttle outputs). Independent
engines are required for V-22 composability.

### Refactored ActuatorEffectivenessHelicopter

The existing single-rotor class is refactored to use `HeliSwashplate` internally.
The public interface and parameter names remain unchanged. The mixing logic
currently inline in `updateSetpoint()` moves into the `HeliSwashplate::mix()` call.

Tail rotor control (yaw compensation) stays in the single-rotor class since it's
specific to single-rotor configurations. Dual-rotor helis have no tail rotor.

### Spool State Machine

Formalized state machine shared by single and dual effectiveness classes:

```
SHUT_DOWN --> GROUND_IDLE --> SPOOLING_UP --> THROTTLE_UNLIMITED
    ^                                              |
    +----------- SPOOLING_DOWN <-------------------+
```

States:
- **SHUT_DOWN**: disarmed, servos may move for pre-flight checks
- **GROUND_IDLE**: armed, throttle at `CA_HELI_THR_IDLE`, swashplate active
- **SPOOLING_UP**: throttle ramping from idle to flight over `COM_SPOOLUP_TIME`
- **THROTTLE_UNLIMITED**: normal flight, full collective range
- **SPOOLING_DOWN**: throttle ramping down after disarm

State is published via `helicopter_status` uORB topic (new message definition in
`msg/HelicopterStatus.msg`) so commander and flight modes can react. Fields:
spool_state (uint8), throttle_output (float32), rpm_measured (float32),
timestamp (uint64).

### V-22 Composability

The dual helicopter code is designed to compose with PX4's existing tiltrotor
VTOL framework. Future `ActuatorEffectivenessHelicopterDualTiltrotor`:

```cpp
class ActuatorEffectivenessHelicopterDualTiltrotor : public ActuatorEffectiveness {
    HeliSwashplate _swashplate1;
    HeliSwashplate _swashplate2;
    ActuatorEffectivenessTilts _tilts;
    ActuatorEffectivenessControlSurfaces _control_surfaces;

    int numMatrices() const override { return 2; }
    // Matrix 0: heli mixing + tilts (MC mode)
    // Matrix 1: control surfaces (FW mode)
};
```

Key design constraints that enable this:
- `HeliSwashplate` writes to a slice of the actuator vector, not the full vector
- Throttle outputs occupy specific indices, leaving room for tilt servos and surfaces
- The dual mixing class does not assume exclusive ownership of the actuator vector
- The VTOL transition state machine in `vtol_att_control` handles mode blending

## Parameters

### Existing (unchanged)

| Parameter | Description |
|-----------|------------|
| `CA_AIRFRAME` | Extended with values 13, 14 |
| `CA_SP0_COUNT` | First swashplate servo count |
| `CA_SP0_ANG0..3` | First swashplate servo angles |
| `CA_SP0_ARM_L0..3` | First swashplate arm lengths |
| `CA_SV_CS0..3_TRIM` | First swashplate servo trims |
| `CA_HELI_THR_C0..4` | Throttle curve (5-point) |
| `CA_HELI_PITCH_C0..4` | Collective pitch curve (5-point) |
| `CA_MAX_SVO_THROW` | Servo linearization angle |
| `COM_SPOOLUP_TIME` | Spoolup duration |

### New for dual

| Parameter | Range | Default | Description |
|-----------|-------|---------|------------|
| `CA_SP1_COUNT` | 2-4 | 3 | Second swashplate servo count |
| `CA_SP1_ANG0..3` | 0-360 | 0/140/220 | Second swashplate servo angles |
| `CA_SP1_ARM_L0..3` | 0-10 | 1.0 | Second swashplate arm lengths |
| `CA_SV_CS4..7_TRIM` | -1..1 | 0 | Second swashplate servo trims |
| `CA_HELI_DCP_SC` | 0-1 | 0.25 | Differential collective pitch scaler |
| `CA_HELI_YAW_SC` | 0-2 | 1.0 | Yaw differential cyclic scaler |
| `CA_HELI_DUAL_ENG` | 0-1 | 0 | 0=single engine, 1=independent |

### New for spool state

| Parameter | Range | Default | Description |
|-----------|-------|---------|------------|
| `CA_HELI_THR_IDLE` | 0-0.3 | 0.05 | Idle throttle output |

## Actuator Output Mapping

### Single rotor (unchanged)

| Index | Function |
|-------|----------|
| 0-3 | Swashplate servos |
| 4 | Main motor throttle |
| 5 | Tail motor / yaw servo |

### Dual rotor

| Index | Function |
|-------|----------|
| 0-3 | Swashplate 1 servos (up to 4) |
| 4-7 | Swashplate 2 servos (up to 4) |
| 8 | Throttle 1 (or single throttle) |
| 9 | Throttle 2 (if independent engines) |

For V-22 composability, tilt servos and control surfaces start at index 10.

## Testing

### Unit tests

**HeliSwashplate:**
- Verify servo outputs for known roll/pitch/collective inputs
- Test 3-servo and 4-servo configurations
- Test linearization on/off
- Test saturation detection

**Dual mixing:**
- Tandem: pure pitch -> differential collective, zero cyclic cross-coupling
- Tandem: pure yaw -> differential cyclic, zero collective
- Transverse: pure roll -> differential collective
- Transverse: pure yaw -> differential cyclic
- Combined inputs -> correct superposition
- DCP scaler effect
- Single-engine vs dual-engine throttle routing

**Spool state:**
- State transitions through full lifecycle
- Throttle limiting during spoolup
- Servo passthrough during ground idle

### SITL integration

- Gazebo models for tandem and transverse dual helicopters
- Airframe configs in ROMFS
- Basic stabilize and altitude hold flight tests

### Regression

- All existing single-rotor helicopter tests pass unchanged after HeliSwashplate
  extraction refactor
- Existing `16001_helicopter` airframe behavior is identical

## Build Sequence

1. Extract `HeliSwashplate` from existing single-rotor code, refactor
   `ActuatorEffectivenessHelicopter` to use it. All existing tests must pass.
2. Add spool state machine, `helicopter_status` uORB topic, idle throttle param.
3. Implement `ActuatorEffectivenessHelicopterDual` with tandem and transverse mixing.
4. Add CA_AIRFRAME enum values and ControlAllocator instantiation cases.
5. Add unit tests for HeliSwashplate, dual mixing, and spool states.
6. Add ROMFS airframe configs for dual tandem and transverse.
7. Create Gazebo SITL models and validate in simulation.
