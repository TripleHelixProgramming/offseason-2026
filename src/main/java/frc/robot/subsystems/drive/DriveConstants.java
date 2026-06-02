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

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);

  // Robot physical dimensions
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
  public static final Distance wheelRadius = Inches.of(2);
  public static final double wheelRadiusMeters = wheelRadius.in(Meters);
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
      2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
  // Wheel Rad/Sec

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
  public static final double turnMotorReduction = (32.0 / 15.0) * (60.0 / 10.0); // SDS MK4
  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Absolute turn encoder configuration
  public static final boolean turnEncoderInverted = false;

  // PathPlanner configuration
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

  public static double driveKp = 10;
  public static double driveKd = 0;
  public static double driveKs = 0;
  public static double driveKv = 0.124;

  // Turn PID configuration
  public static final double turnKp = 2.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // TorqueCurrent peak at which the wheels start to slip; used for slip detection in
  // TorqueCurrentFOC control mode. This needs to be tuned to your individual robot.
  static final int kSlipCurrent = 120;

  // Stator current limit for azimuth (steer) motors; lower than drive to reduce brownout risk
  // since steering requires minimal torque compared to driving.
  static final int kSteerStatorCurrentLimit = 60;

  static final double ODOMETRY_FREQUENCY = 100.0; // Hz

  /**
   * Creates a CommandSwerveDrivetrain instance. This should only be called once in your robot
   * program,.
   */
  //   public static CommandSwerveDrivetrain createDrivetrain() {
  //     return new CommandSwerveDrivetrain(
  //         DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
  //   }

  /** Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types. */
  public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules Constants for each specific module
     */
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
      super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set
     *     to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param modules Constants for each specific module
     */
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          TalonFX::new,
          TalonFX::new,
          CANcoder::new,
          drivetrainConstants,
          odometryUpdateFrequency,
          modules);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set
     *     to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
     *     [x, y, theta]ᵀ, with units in meters and radians
     * @param visionStandardDeviation The standard deviation for vision calculation in the form [x,
     *     y, theta]ᵀ, with units in meters and radians
     * @param modules Constants for each specific module
     */
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          TalonFX::new,
          TalonFX::new,
          CANcoder::new,
          drivetrainConstants,
          odometryUpdateFrequency,
          odometryStandardDeviation,
          visionStandardDeviation,
          modules);
    }
  }
}
