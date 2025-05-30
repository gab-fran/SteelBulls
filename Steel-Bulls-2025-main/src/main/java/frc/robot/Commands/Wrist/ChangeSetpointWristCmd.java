package frc.robot.Commands.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;

public class ChangeSetpointWristCmd extends Command {

  public double wristSetpoint;

  public ChangeSetpointWristCmd(double kWristPosition) {
    wristSetpoint = kWristPosition;
  }

  @Override
  public void execute() {
    WristConstants.kWristMotorSetPoint = wristSetpoint;
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
