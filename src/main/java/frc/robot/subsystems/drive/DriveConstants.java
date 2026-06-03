// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.Constants.MotorConstants.NEOConstants;

public class DriveConstants {

  public static final String zeroRotationKey = "ZeroRotation";

  // Robot physical dimensions
  // TODO: Update wheelbase and track width
  public static final Distance wheelBase = Inches.of(22.5);
  public static final Distance trackWidth = Inches.of(19.5);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(wheelBase.div(2.0), trackWidth.div(2.0)),
        new Translation2d(wheelBase.div(2.0), trackWidth.div(-2.0)),
        new Translation2d(wheelBase.div(-2.0), trackWidth.div(2.0)),
        new Translation2d(wheelBase.div(-2.0), trackWidth.div(-2.0))
      };
  public static final Distance driveBaseRadius =
      Meters.of(Translation2d.kZero.getDistance(moduleTranslations[0]));

  // Drive motor configuration
  public static final Distance wheelRadius = Inches.of(1.5);
  public static final double wheelRadiusMeters = wheelRadius.in(Meters);

  // TODO: Update drive motor reduction
  public static final double driveMotorReduction =
      (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // SDS MK4 L2
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);
  public static final LinearVelocity drivetrainSpeedLimit =
      MetersPerSecond.of(
          0.9
              * (wheelRadius.in(Meters) * 2.0 * Math.PI)
              * NEOConstants.kFreeSpeed.in(RotationsPerSecond)
              / driveMotorReduction);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Chassis movement limits
  private static final LinearVelocity driverSpeedLimit = MetersPerSecond.of(5);
  public static final LinearVelocity maxChassisVelocity =
      MetersPerSecond.of(
          Math.min(drivetrainSpeedLimit.in(MetersPerSecond), driverSpeedLimit.in(MetersPerSecond)));
  public static final LinearAcceleration maxChassisAcceleration = MetersPerSecondPerSecond.of(3.0);

  public static final AngularVelocity maxChassisAngularVelocity =
      RadiansPerSecond.of(maxChassisVelocity.in(MetersPerSecond) / driveBaseRadius.in(Meters));
  public static final AngularAcceleration maxChassisAngularAcceleration =
      RadiansPerSecondPerSecond.of(30);

  public static final PathConstraints pathFollowingConstraints =
      new PathConstraints(
          maxChassisVelocity.in(MetersPerSecond),
          maxChassisAcceleration.in(MetersPerSecondPerSecond),
          maxChassisAngularVelocity.in(RadiansPerSecond),
          maxChassisAngularAcceleration.in(RadiansPerSecondPerSecond));

  // Turn motor configuration
  public static final boolean turnInverted = false;
  // TODO: Update turn motor reduction
  public static final double turnMotorReduction = (32.0 / 15.0) * (60.0 / 10.0); // SDS MK4
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Absolute turn encoder configuration
  public static final boolean turnEncoderInverted = false;

  // Module controller gains
  public static final double driveKp = 10.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.124;

  public static final double turnKp = 2.0;
  public static final double turnKd = 0.0;

  public static final double turnPIDMinInput = 0.0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  // TODO: Update mass and MOI
  public static final Mass robotMass = Pounds.of(150);
  public static final MomentOfInertia robotMOI = KilogramSquareMeters.of(6);
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMass.in(Kilograms),
          robotMOI.in(KilogramSquareMeters),
          new ModuleConfig(
              wheelRadius.in(Meters),
              drivetrainSpeedLimit.in(MetersPerSecond),
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              NEOConstants.kDefaultSupplyCurrentLimit,
              1),
          moduleTranslations);

  // TorqueCurrent peak at which the wheels start to slip; used for slip detection in
  // TorqueCurrentFOC control mode. This needs to be tuned to your individual robot.
  static final int kSlipCurrent = 120;

  // Stator current limit for azimuth (steer) motors; lower than drive to reduce brownout risk
  // since steering requires minimal torque compared to driving.
  static final int kSteerStatorCurrentLimit = 60;

  static final double ODOMETRY_FREQUENCY = 100.0; // Hz
}
