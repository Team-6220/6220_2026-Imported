// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import java.util.HashMap;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class ShooterConstants {
  // first number is distance
  // first number of array is rpm
  // second number of array is angler angle
  // third number of array is first-shot boost multiplier
  public static HashMap<Double, Double[]> rpmAngle =
      new HashMap<Double, Double[]>() {
        {
          put(-1.0, new Double[] {100.0, 0.0, 0.0}); // Testing purposes
          put(0.0, new Double[] {1900.0, 0.0, 1.15}); // Testing purposes
          put(0.4, new Double[] {1900.0, 0.0, 1.15});
          put(0.6, new Double[] {1900.0, 0.0, 1.15});
          put(0.8, new Double[] {1900.0, 0.0, 1.19});
          put(1.0, new Double[] {2050.0, 2.6, 1.2});
          put(1.2, new Double[] {2040.0, 5.85, 1.21});
          put(1.4, new Double[] {2060.0, 6.00, 1.22});
          put(1.6, new Double[] {2040.0, 8.0, 1.23});
          put(1.8, new Double[] {2000.0, 10.0, 1.24});
          put(2.0, new Double[] {1980.0, 12.61, 1.25});
          put(2.2, new Double[] {2000.0, 13.4, 1.26});
          put(2.4, new Double[] {2020.0, 14.10, 1.27});
          put(2.6, new Double[] {2040.0, 15.43, 1.28});
          put(2.8, new Double[] {2060.0, 16.47, 1.29});
          put(3.0, new Double[] {2080.0, 17.37, 1.3});
          put(3.2, new Double[] {2100.0, 18.30, 1.31});
          put(3.4, new Double[] {2120.0, 19.4, 1.32});
          put(3.6, new Double[] {2140.0, 20.45, 1.33});
          put(3.8, new Double[] {2160.0, 21.49, 1.34});
          put(4.0, new Double[] {2200.0, 22.5, 1.30});
          put(4.2, new Double[] {2220.0, 23.5, 1.31});
          put(4.4, new Double[] {2240.0, 24.5, 1.32});
          // Extrapolated from the 3.8-4.4 trend (+20 RPM, +1.0 deg, +0.01 boost per 0.2 m).
          // Untested guesses: tune on the field.
          put(4.6, new Double[] {2260.0, 25.5, 1.33});
          put(4.8, new Double[] {2280.0, 26.5, 1.34});
          put(5.0, new Double[] {2300.0, 27.5, 1.35});
          put(5.2, new Double[] {2320.0, 28.5, 1.36});
          put(5.4, new Double[] {2340.0, 29.5, 1.37});
          put(5.6, new Double[] {2360.0, 30.5, 1.38});
          put(5.8, new Double[] {2380.0, 31.5, 1.39});
          put(6.0, new Double[] {2400.0, 32.5, 1.40});
        }
      };

  // Same layout as rpmAngle, but the distance is from the robot to the shuttle target (see
  // HubTargeting.getShuttleTarget()). Starts as a copy of rpmAngle; tune it for passing.
  public static HashMap<Double, Double[]> shuttleRpmAngle =
      new HashMap<Double, Double[]>() {
        {
          put(-1.0, new Double[] {100.0, 0.0, 0.0}); // Testing purposes
          put(0.0, new Double[] {1900.0, 0.0, 1.15}); // Testing purposes
          put(0.4, new Double[] {1900.0, 0.0, 1.15});
          put(0.6, new Double[] {1900.0, 0.0, 1.15});
          put(0.8, new Double[] {1900.0, 0.0, 1.19});
          put(1.0, new Double[] {2050.0, 2.6, 1.2});
          put(1.2, new Double[] {2040.0, 5.85, 1.21});
          put(1.4, new Double[] {2060.0, 6.00, 1.22});
          put(1.6, new Double[] {2040.0, 8.0, 1.23});
          put(1.8, new Double[] {2000.0, 10.0, 1.24});
          put(2.0, new Double[] {1980.0, 12.61, 1.25});
          put(2.2, new Double[] {2000.0, 13.4, 1.26});
          put(2.4, new Double[] {2020.0, 14.10, 1.27});
          put(2.6, new Double[] {2040.0, 15.43, 1.28});
          put(2.8, new Double[] {2060.0, 16.47, 1.29});
          put(3.0, new Double[] {2080.0, 17.37, 1.3});
          put(3.2, new Double[] {2100.0, 18.30, 1.31});
          put(3.4, new Double[] {2120.0, 19.4, 1.32});
          put(3.6, new Double[] {2140.0, 20.45, 1.33});
          put(3.8, new Double[] {2160.0, 21.49, 1.34});
          put(4.0, new Double[] {2200.0, 22.5, 1.30});
          put(4.2, new Double[] {2220.0, 23.5, 1.31});
          put(4.4, new Double[] {2240.0, 24.5, 1.32});
          // Extrapolated from the 3.8-4.4 trend (+20 RPM, +1.0 deg, +0.01 boost per 0.2 m).
          // Untested guesses: tune on the field.
          put(4.6, new Double[] {2260.0, 25.5, 1.33});
          put(4.8, new Double[] {2280.0, 26.5, 1.34});
          put(5.0, new Double[] {2300.0, 27.5, 1.35});
          put(5.2, new Double[] {2320.0, 28.5, 1.36});
          put(5.4, new Double[] {2340.0, 29.5, 1.37});
          put(5.6, new Double[] {2360.0, 30.5, 1.38});
          put(5.8, new Double[] {2380.0, 31.5, 1.39});
          put(6.0, new Double[] {2400.0, 32.5, 1.40});
        }
      };

  // ========== Toggles ==========
  /**
   * When true, RB and B aim the robot at the shuttle target and pick RPM/angle from {@link
   * #shuttleRpmAngle} by pose distance. When false, they fire PassToAlliance's fixed RPM/angle.
   */
  public static final boolean AUTO_SHUTTLE_ENABLED = true;

  /**
   * When true, shots blend between the two nearest table entries instead of rounding the distance
   * to the nearest 0.2 m key. Past either end of a table, the end value is held.
   */
  public static final boolean INTERPOLATION_ENABLED = true;

  // ========== Shuttle target ==========
  // The shuttle target sits on our side of the field (the half the robot is on, by y), on the line
  // between the trench and the bump, inside our alliance zone.
  /** Field length (m), 2026 welded field. Red alliance wall is at this x. */
  public static final double FIELD_LENGTH_M = 16.541;

  /**
   * Sideways distance (m) from the hub's center line (y = 4.0346) to the trench/bump boundary.
   * Half the hub (23.5 in) plus one bump (73 in) from the 2026 game manual. Check on the field.
   */
  public static final double SHUTTLE_TARGET_Y_OFFSET_M = 2.45;

  /** Distance (m) from our alliance wall to the shuttle target. Tune for where passes land. */
  public static final double SHUTTLE_TARGET_X_FROM_WALL_M = 2.5;

  // ========== Interpolation ==========
  // Built from the tables above. Skips negative test keys: interpolating toward the -1.0 entry
  // would bend real short shots.
  public static final InterpolatingDoubleTreeMap RPM = new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap ANGLE = new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap BOOST = new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap SHUTTLE_RPM = new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap SHUTTLE_ANGLE = new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap SHUTTLE_BOOST = new InterpolatingDoubleTreeMap();

  static {
    fillInterpolationMaps(rpmAngle, RPM, ANGLE, BOOST);
    fillInterpolationMaps(shuttleRpmAngle, SHUTTLE_RPM, SHUTTLE_ANGLE, SHUTTLE_BOOST);
  }

  private static void fillInterpolationMaps(
      HashMap<Double, Double[]> table,
      InterpolatingDoubleTreeMap rpm,
      InterpolatingDoubleTreeMap angle,
      InterpolatingDoubleTreeMap boost) {
    for (var entry : table.entrySet()) {
      if (entry.getKey() < 0) continue;
      rpm.put(entry.getKey(), entry.getValue()[0]);
      angle.put(entry.getKey(), entry.getValue()[1]);
      boost.put(entry.getKey(), entry.getValue()[2]);
    }
  }

  public static final double topTESTrpm = 1600.0;

  public static final double bottomTESTrpm = 1200;

  // ========== First Shot Boost Configuration ==========
  /** Multiplier for first shot RPM (e.g., 1.15 for a 15% boost). */
  public static double FIRST_SHOT_BOOST_MULTIPLIER = 1.25;

  /**
   * @deprecated Use {@link #FIRST_SHOT_BOOST_MULTIPLIER} instead.
   */
  @Deprecated public static double FIRST_SHOT_BOOST_PERCENT = FIRST_SHOT_BOOST_MULTIPLIER;

  /** RPM dip threshold to detect when shot has left the shooter (in RPM) */
  public static final double RPM_DIP_THRESHOLD = 300.0;
}
