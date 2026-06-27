// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class AllianceSelector {

  private final AllianceSelectorIO io;
  private final AllianceSelectorIOInputsAutoLogged inputs =
      new AllianceSelectorIOInputsAutoLogged();

  private EventLoop eventLoop = new EventLoop();
  private BooleanEvent changedAlliance;
  private BooleanEvent agreementInAllianceInputs;

  public AllianceSelector(int port) {
    io = new AllianceSelectorIO(port);
    changedAlliance = new BooleanEvent(eventLoop, () -> inputs.allianceChanged);
    agreementInAllianceInputs = new BooleanEvent(eventLoop, () -> inputs.agreementInAllianceInputs);
  }

  /**
   * @return Whether the field is rotated from the driver's perspective
   */
  public boolean fieldRotated() {
    return inputs.allianceFromSwitch.equals(Alliance.Red);
  }

  /**
   * @return The current alliance
   */
  public Alliance getAllianceColor() {
    return inputs.allianceFromSwitch;
  }

  /**
   * @return Object for binding a command to a change in alliance color
   */
  public Trigger getAllianceColorChange() {
    return changedAlliance.castTo(Trigger::new);
  }

  /**
   * @return Object for binding a command to agreement between the sources of information for
   *     alliance color
   */
  public Trigger getAgreementInAllianceColor() {
    return agreementInAllianceInputs.castTo(Trigger::new);
  }

  public void disabledPeriodic() {
    eventLoop.poll();
    io.updateInputs(inputs);
    Logger.processInputs("AllianceSelector", inputs);
  }
}
