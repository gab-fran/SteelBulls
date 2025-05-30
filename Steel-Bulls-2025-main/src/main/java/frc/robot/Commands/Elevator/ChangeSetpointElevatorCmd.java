package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;

public class ChangeSetpointElevatorCmd extends Command {

  public double elevatorSetpoint;

  public ChangeSetpointElevatorCmd(double kElevatorPosition) {
    elevatorSetpoint = kElevatorPosition;
  }

  @Override
  public void execute() {
    ElevatorConstants.kElevatorMotorSetPoint = elevatorSetpoint;
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
