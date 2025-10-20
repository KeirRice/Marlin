/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/**
 * scara.cpp
 */

#include "../inc/MarlinConfig.h"

#if IS_SCARA

#include "scara.h"
#include "motion.h"
#include "planner.h"

#if ENABLED(AXEL_TPARA)
  #include "endstops.h"
  #include "../MarlinCore.h"
#endif

float segments_per_second = DEFAULT_SEGMENTS_PER_SECOND;

#if ANY(MORGAN_SCARA, MP_SCARA)

  static constexpr xy_pos_t scara_offset = { SCARA_OFFSET_X, SCARA_OFFSET_Y };

  /**
   * Morgan SCARA Forward Kinematics. Results in 'cartes'.
   * Maths and first version by QHARLEY.
   * Integrated into Marlin and slightly restructured by Joachim Cerny.
   */
  void forward_kinematics(const_float_t a, const_float_t b) {
    const float a_sin = sin(RADIANS(a)) * L1,
                a_cos = cos(RADIANS(a)) * L1,
                b_sin = sin(RADIANS(SUM_TERN(MP_SCARA, b, a))) * L2,
                b_cos = cos(RADIANS(SUM_TERN(MP_SCARA, b, a))) * L2;

    cartes.x = a_cos + b_cos + scara_offset.x;  // theta
    cartes.y = a_sin + b_sin + scara_offset.y;  // phi

    /*
      DEBUG_ECHOLNPGM(
        "SCARA FK Angle a=", a,
        " b=", b,
        " a_sin=", a_sin,
        " a_cos=", a_cos,
        " b_sin=", b_sin,
        " b_cos=", b_cos
      );
      DEBUG_ECHOLNPGM(" cartes (X,Y) = "(cartes.x, ", ", cartes.y, ")");
    //*/
  }

#endif

#if ENABLED(MORGAN_SCARA)

  void scara_set_axis_is_at_home(const AxisEnum axis) {
    if (axis == Z_AXIS)
      current_position.z = Z_HOME_POS;
    else {
      // MORGAN_SCARA uses a Cartesian XY home position
      xyz_pos_t homeposition = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS };
      //DEBUG_ECHOLNPGM_P(PSTR("homeposition X"), homeposition.x, SP_Y_LBL, homeposition.y);

      delta = homeposition;
      forward_kinematics(delta.a, delta.b);
      current_position[axis] = cartes[axis];

      //DEBUG_ECHOLNPGM_P(PSTR("Cartesian X"), current_position.x, SP_Y_LBL, current_position.y);
      update_software_endstops(axis);
    }
  }

  /**
   * Morgan SCARA Inverse Kinematics. Results are stored in 'delta'.
   *
   * See https://reprap.org/forum/read.php?185,283327
   *
   * Maths and first version by QHARLEY.
   * Integrated into Marlin and slightly restructured by Joachim Cerny.
   */
  void inverse_kinematics(const xyz_pos_t &raw) {
    float C2, S2, SK1, SK2, THETA, PSI;

    // Translate SCARA to standard XY with scaling factor
    const xy_pos_t spos = raw - scara_offset;

    const float H2 = HYPOT2(spos.x, spos.y);
    if (L1 == L2)
      C2 = H2 / L1_2_2 - 1;
    else
      C2 = (H2 - (L1_2 + L2_2)) / (2.0f * L1 * L2);

    LIMIT(C2, -1, 1);

    S2 = SQRT(1.0f - sq(C2));

    // Unrotated Arm1 plus rotated Arm2 gives the distance from Center to End
    SK1 = L1 + L2 * C2;

    // Rotated Arm2 gives the distance from Arm1 to Arm2
    SK2 = L2 * S2;

    // Angle of Arm1 is the difference between Center-to-End angle and the Center-to-Elbow
    THETA = ATAN2(SK1, SK2) - ATAN2(spos.x, spos.y);

    // Angle of Arm2
    PSI = ATAN2(S2, C2);

    delta.set(DEGREES(THETA), DEGREES(SUM_TERN(MORGAN_SCARA, PSI, THETA)), raw.z);

    /*
      DEBUG_POS("SCARA IK", raw);
      DEBUG_POS("SCARA IK", delta);
      DEBUG_ECHOLNPGM("  SCARA (x,y) ", sx, ",", sy, " C2=", C2, " S2=", S2, " Theta=", THETA, " Psi=", PSI);
    //*/
  }

#endif // MORGAN_SCARA

#if ENABLED(MP_SCARA) && ENABLED(SCARA_DUAL_CONTROL)

// SCARA Dual Control System - Simplified Approach
// No mode switching needed - smart G-code handling

// Global state for SCARA dual control
struct SCARAState {
  float theta1_unwrapped;  // Unwrapped angle A (continuous rotation)
  float theta2_unwrapped;  // Unwrapped angle B (continuous rotation)
  bool at_singularity;     // True when at origin (0,0)
  float last_valid_theta1; // Last valid angle A before singularity
  float last_valid_theta2; // Last valid angle B before singularity
  bool initialized;        // True when state is initialized
} scara_state = {0, 0, false, 0, 0, false};

// Initialize SCARA state
void init_scara_state() {
  if (!scara_state.initialized) {
    scara_state.theta1_unwrapped = SCARA_OFFSET_THETA1;
    scara_state.theta2_unwrapped = SCARA_OFFSET_THETA2;
    scara_state.last_valid_theta1 = SCARA_OFFSET_THETA1;
    scara_state.last_valid_theta2 = SCARA_OFFSET_THETA2;
    scara_state.at_singularity = false;
    scara_state.initialized = true;
  }
}

// Get display angle (wrapped to 0-360 for user display)
float get_display_theta1() {
  return fmod(scara_state.theta1_unwrapped + 360.0f, 360.0f);
}

float get_display_theta2() {
  return fmod(scara_state.theta2_unwrapped + 360.0f, 360.0f);
}

// Direct angle control functions
void set_scara_angles(const float theta1, const float theta2) {
  scara_state.theta1_unwrapped = theta1;
  scara_state.theta2_unwrapped = theta2;
  scara_state.last_valid_theta1 = theta1;
  scara_state.last_valid_theta2 = theta2;
  
  // Update delta for motion planning
  delta.set(theta1, theta2, current_position.z);
  
  // Update Cartesian position for tracking
  forward_kinematics(theta1, theta2);
  current_position.x = cartes.x;
  current_position.y = cartes.y;
  sync_plan_position();
}

void set_scara_angle_a(const float theta1) {
  scara_state.theta1_unwrapped = theta1;
  scara_state.last_valid_theta1 = theta1;
  
  // Update delta for motion planning
  delta.set(theta1, scara_state.theta2_unwrapped, current_position.z);
  
  // Update Cartesian position for tracking
  forward_kinematics(theta1, scara_state.theta2_unwrapped);
  current_position.x = cartes.x;
  current_position.y = cartes.y;
  sync_plan_position();
}

void set_scara_angle_b(const float theta2) {
  scara_state.theta2_unwrapped = theta2;
  scara_state.last_valid_theta2 = theta2;
  
  // Update delta for motion planning
  delta.set(scara_state.theta1_unwrapped, theta2, current_position.z);
  
  // Update Cartesian position for tracking
  forward_kinematics(scara_state.theta1_unwrapped, theta2);
  current_position.x = cartes.x;
  current_position.y = cartes.y;
  sync_plan_position();
}

// Get current angles
float get_scara_angle_a() {
  return scara_state.theta1_unwrapped;
}

float get_scara_angle_b() {
  return scara_state.theta2_unwrapped;
}

// Enhanced inverse kinematics for dual control
void inverse_kinematics(const xyz_pos_t &raw) {
  // Initialize state if needed
  init_scara_state();
  
  const float x = raw.x - SCARA_OFFSET_X;
  const float y = raw.y - SCARA_OFFSET_Y;
  const float c = HYPOT(x, y);
  
  // Check for singularity at origin
  if (c < SCARA_SINGULARITY_THRESHOLD) {
    scara_state.at_singularity = true;
    // Maintain current angles when at singularity
    delta.set(scara_state.theta1_unwrapped, scara_state.theta2_unwrapped, raw.z);
    return;
  }
  
  scara_state.at_singularity = false;
  
  // Check reachability
  if (c > SCARA_MAX_REACH) {
    SERIAL_ECHOLNPGM("SCARA: Position unreachable (", c, "mm > ", SCARA_MAX_REACH, "mm)");
    return;
  }
  
  // For equal-length arms, use simplified law of cosines
  const float cos_angle = c / (2.0f * L1);
  
  if (cos_angle > 1.0f) {
    SERIAL_ECHOLNPGM("SCARA: Position unreachable (arms too short)");
    return;
  }
  
  const float angle = acos(cos_angle);
  const float theta1 = atan2(y, x) - angle;
  const float theta2 = 2.0f * angle;
  
  // Choose solution closest to current unwrapped angles for smooth motion
  const float theta1_alt = atan2(y, x) + angle;
  const float theta2_alt = -2.0f * angle;
  
  const float dist1 = fabs(theta1 - scara_state.theta1_unwrapped) + fabs(theta2 - scara_state.theta2_unwrapped);
  const float dist2 = fabs(theta1_alt - scara_state.theta1_unwrapped) + fabs(theta2_alt - scara_state.theta2_unwrapped);
  
  if (dist2 < dist1) {
    scara_state.theta1_unwrapped = theta1_alt;
    scara_state.theta2_unwrapped = theta2_alt;
  } else {
    scara_state.theta1_unwrapped = theta1;
    scara_state.theta2_unwrapped = theta2;
  }
  
  // Update last valid angles
  scara_state.last_valid_theta1 = scara_state.theta1_unwrapped;
  scara_state.last_valid_theta2 = scara_state.theta2_unwrapped;
  
  // Set delta for motion planning
  delta.set(scara_state.theta1_unwrapped, scara_state.theta2_unwrapped, raw.z);
}

// Enhanced position reporting
void scara_report_positions() {
  SERIAL_ECHOPGM("SCARA A:", get_display_theta1(), " B:", get_display_theta2());
  SERIAL_ECHOPGM(" X:", current_position.x, " Y:", current_position.y, " Z:", current_position.z);
  if (scara_state.at_singularity) {
    SERIAL_ECHOPGM(" (SINGULARITY)");
  }
  SERIAL_EOL();
}

// Home continuous SCARA (no physical endstops needed)
void home_continuous_scara() {
  init_scara_state();
  
  // Set home position angles
  scara_state.theta1_unwrapped = SCARA_OFFSET_THETA1;
  scara_state.theta2_unwrapped = SCARA_OFFSET_THETA2;
  scara_state.last_valid_theta1 = SCARA_OFFSET_THETA1;
  scara_state.last_valid_theta2 = SCARA_OFFSET_THETA2;
  scara_state.at_singularity = false;
  
  // Update delta for motion planning
  delta.set(scara_state.theta1_unwrapped, scara_state.theta2_unwrapped, current_position.z);
  
  // Update Cartesian position using forward kinematics
  forward_kinematics(scara_state.theta1_unwrapped, scara_state.theta2_unwrapped);
  current_position.x = cartes.x;
  current_position.y = cartes.y;
  
  // Sync with planner
  sync_plan_position();
  
  SERIAL_ECHOLNPGM("SCARA: Homed to A:", get_display_theta1(), " B:", get_display_theta2());
}

#endif // MP_SCARA && SCARA_DUAL_CONTROL

#endif // IS_SCARA
