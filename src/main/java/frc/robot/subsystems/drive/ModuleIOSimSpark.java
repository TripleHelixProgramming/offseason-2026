package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Robot;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;

/** Module IO implementation using REV Spark sim layer for hardware-in-the-loop style simulation. */
public class ModuleIOSimSpark implements ModuleIO {

  private Rotation2d zeroRotation = Rotation2d.kZero;

  private final SparkMax driveSpark;
  private final SparkMax turnSpark;
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  private final SparkMaxSim driveSparkSim;
  private final SparkMaxSim turnSparkSim;
  private final DCMotorSim driveDCMotorSim;
  private final DCMotorSim turnDCMotorSim;

  public ModuleIOSimSpark(ModuleConstants constants) {
    driveSpark = new SparkMax(constants.driveCanId(), MotorType.kBrushless);
    turnSpark = new SparkMax(constants.turnCanId(), MotorType.kBrushless);
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    driveSpark.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveSparkSim = new SparkMaxSim(driveSpark, driveGearbox);
    turnSparkSim = new SparkMaxSim(turnSpark, turnGearbox);

    driveDCMotorSim = DriveConstants.createDriveSim();
    turnDCMotorSim = DriveConstants.createTurnSim();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    double busVoltage = RoboRioSim.getVInVoltage();

    // Update drive simulation state
    driveDCMotorSim.setInput(driveSparkSim.getAppliedOutput() * busVoltage);
    driveDCMotorSim.update(Robot.defaultPeriodSecs);
    driveSparkSim.iterate(
        driveDCMotorSim.getAngularVelocityRadPerSec(), busVoltage, Robot.defaultPeriodSecs);

    // Update turn simulation state
    turnDCMotorSim.setInput(turnSparkSim.getAppliedOutput() * busVoltage);
    turnDCMotorSim.update(Robot.defaultPeriodSecs);
    turnSparkSim.iterate(
        turnDCMotorSim.getAngularVelocityRadPerSec(), busVoltage, Robot.defaultPeriodSecs);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSparkSim.getPosition();
    inputs.driveVelocityRadPerSec = driveSparkSim.getVelocity();
    inputs.driveAppliedVolts = driveSparkSim.getAppliedOutput() * busVoltage;
    inputs.driveCurrentAmps = Math.abs(driveSparkSim.getMotorCurrent());

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnZero = zeroRotation;
    inputs.turnPosition = new Rotation2d(turnSparkSim.getPosition()).minus(zeroRotation);
    inputs.turnVelocityRadPerSec = turnSparkSim.getVelocity();
    inputs.turnAppliedVolts = turnSparkSim.getAppliedOutput() * busVoltage;
    inputs.turnCurrentAmps = Math.abs(turnSparkSim.getMotorCurrent());

    // Update odometry inputs (single sample; high-frequency odometry is unnecessary in sim)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSparkSim.setAppliedOutput(output / 12.0);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSparkSim.setAppliedOutput(output / 12.0);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec, double accelRadPerSec2) {
    double ffVolts =
        driveKs * Math.signum(velocityRadPerSec)
            + driveKv * velocityRadPerSec
            + driveKa * accelRadPerSec2;
    driveController.setSetpoint(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnZero(Rotation2d rotation) {
    zeroRotation = rotation;
  }

  @Override
  public void setTurnPosition(Rotation2d rotation, double velocityRadPerSec) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput);
    double ffVolts = turnKv * velocityRadPerSec;
    turnController.setSetpoint(
        setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVolts, ArbFFUnits.kVoltage);
  }
}
