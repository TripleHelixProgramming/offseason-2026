// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified work Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class FeatureFlags {
    /** Enable to print loop timing when total exceeds 20ms. */
    public static final boolean PROFILING_ENABLED = false;
  }

  public final class RobotConstants {
    public static final double kNominalVoltage = 12.0;
  }

  public static final class MotorConstants {
    public static final class NEOConstants {
      public static final AngularVelocity kFreeSpeed = RPM.of(5676);
      public static final int kDefaultSupplyCurrentLimit = 25;
      public static final int kDefaultStatorCurrentLimit = 50;
    }

    public static final class NEO550Constants {
      public static final AngularVelocity kFreeSpeed = RPM.of(11000);
      public static final int kDefaultSupplyCurrentLimit = 10;
      public static final int kDefaultStatorCurrentLimit = 20;
    }
  }

  public static final class DIOPorts {
    // max length is 8
    public static final int[] autonomousModeSelector = {0, 1, 2};

    public static final int allianceColorSelector = 3;
  }

  public static final class OIPorts {
    public static final int defaultDriver = 0;
    public static final int defaultOperator = 1;
  }

  public static final class CANBusPorts {

    public static final class CAN2 {
      public static final CANBus bus = CANBus.roboRIO();

      public static final int pd = 0;
      public static final int gyro = 0;

      public static final int backLeftDrive = 11;
      public static final int backRightDrive = 24;
      public static final int frontRightDrive = 21;
      public static final int frontLeftDrive = 14;

      public static final int backLeftTurn = 23;
      public static final int backRightTurn = 25;
      public static final int frontRightTurn = 16;
      public static final int frontLeftTurn = 22;
    }
  }
}
