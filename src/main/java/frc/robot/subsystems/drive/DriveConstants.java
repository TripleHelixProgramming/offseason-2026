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
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.Constants.CANBusPorts.CAN2;
import frc.robot.Constants.MotorConstants.NEO550Constants;
import frc.robot.Constants.MotorConstants.NEOConstants;

public class DriveConstants {

  /** Per-module configuration used to instantiate drive and turn Spark MAX controllers. */
  public record ModuleConstants(String name, int driveCanId, int turnCanId) {}

  public static final ModuleConstants frontLeft =
      new ModuleConstants("FrontLeft", CAN2.frontLeftDrive, CAN2.frontLeftTurn);
  public static final ModuleConstants frontRight =
      new ModuleConstants("FrontRight", CAN2.frontRightDrive, CAN2.frontRightTurn);
  public static final ModuleConstants backLeft =
      new ModuleConstants("BackLeft", CAN2.backLeftDrive, CAN2.backLeftTurn);
  public static final ModuleConstants backRight =
      new ModuleConstants("BackRight", CAN2.backRightDrive, CAN2.backRightTurn);

  public static final String zeroRotationKey = "ZeroRotation";

  // Robot physical dimensions
  public static final Distance wheelBase = Inches.of(14.5);
  public static final Distance trackWidth = Inches.of(14.5);
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
  public static final double driveMotorReduction = (22.0 / 12.0) * (45.0 / 15.0);
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);
  public static final LinearVelocity drivetrainSpeedLimit =
      MetersPerSecond.of(
          0.9
              * (wheelRadiusMeters * 2.0 * Math.PI)
              * NEOConstants.kFreeSpeed.in(RotationsPerSecond)
              / driveMotorReduction);

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
  // TODO: Update turn motor reduction
  public static final double turnMotorReduction = 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Module controller gains
  public static final double driveKs = 0.0; // V — characterize with SysId
  // Theoretical kV from motor model (reduction / motor Kv). Refine with SysId.
  public static final double driveKv =
      driveMotorReduction / driveGearbox.KvRadPerSecPerVolt; // V·s/rad
  public static final double driveKa = 0.0; // V·s²/rad — characterize with SysId

  // Theoretical kV from motor model (reduction / motor Kv). Refine with SysId.
  public static final double turnKv =
      turnMotorReduction / turnGearbox.KvRadPerSecPerVolt; // V·s/rad
  public static final double turnKa = 0.0; // V·s²/rad — characterize with SysId

  public static final double turnPIDMinInput = 0.0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final Mass robotMass = Pounds.of(42.55);
  public static final MomentOfInertia robotMOI = KilogramSquareMeters.of(1.5);
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
              NEOConstants.kDefaultStatorCurrentLimit,
              1),
          moduleTranslations);

  static final double ODOMETRY_FREQUENCY = 100.0; // Hz

  // Spark MAX motor configurations
  public static final SparkMaxConfig driveConfig;
  public static final SparkMaxConfig turnConfig;

  // Sim MOI values (kg*m^2)
  public static final double driveSimMOI = 0.025;
  public static final double turnSimMOI = 0.004;

  public static DCMotorSim createDriveSim() {
    return new DCMotorSim(
        LinearSystemId.createDCMotorSystem(driveGearbox, driveSimMOI, driveMotorReduction),
        driveGearbox);
  }

  public static DCMotorSim createTurnSim() {
    return new DCMotorSim(
        LinearSystemId.createDCMotorSystem(turnGearbox, turnSimMOI, turnMotorReduction),
        turnGearbox);
  }

  static {
    driveConfig = new SparkMaxConfig();
    driveConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(NEOConstants.kDefaultStatorCurrentLimit)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(2 * Math.PI / driveMotorReduction)
        .velocityConversionFactor((2 * Math.PI) / 60.0 / driveMotorReduction)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.004, 0.0, 0.0);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(NEO550Constants.kDefaultStatorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .absoluteEncoder
        .inverted(true)
        .positionConversionFactor(2 * Math.PI)
        .velocityConversionFactor((2 * Math.PI) / 60.0)
        .averageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .pid(0.1, 0.0, 0.0);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
  }
}
