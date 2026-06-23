// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/** Physics sim implementation of module IO. */
public class ModuleIOSimWPI implements ModuleIO {

  private static final double DRIVE_KP = 0.05;
  private static final double DRIVE_KD = 0;
  private static final double DRIVE_KS = 0;
  private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / 0.91035);
  private static final double DRIVE_KA = driveKa;

  private static final double TURN_KP = 8.0;
  private static final double TURN_KD = 0.0;
  private static final double TURN_KV = turnKv;

  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
  private PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
  private double driveFFVolts = 0.0;
  private double turnFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSimWPI() {
    // Create drive and turn sim models
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveGearbox, 0.025, driveMotorReduction),
            driveGearbox);
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnGearbox, 0.004, turnMotorReduction),
            turnGearbox);

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnFFVolts + turnController.calculate(turnSim.getAngularPositionRad());
    } else {
      turnController.reset();
    }

    // Update simulation state
    double busVoltage = RoboRioSim.getVInVoltage();
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -busVoltage, busVoltage));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -busVoltage, busVoltage));
    driveSim.update(0.02);
    turnSim.update(0.02);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
    // matter)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec, double accelRadPerSec2) {
    driveClosedLoop = true;
    driveFFVolts =
        DRIVE_KS * Math.signum(velocityRadPerSec)
            + DRIVE_KV * velocityRadPerSec
            + DRIVE_KA * accelRadPerSec2;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation, double velocityRadPerSec) {
    turnClosedLoop = true;
    turnFFVolts = TURN_KV * velocityRadPerSec;
    turnController.setSetpoint(rotation.getRadians());
  }
}
