package frc.robot.Commands.Pendulum;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Pendulum.PendulumSubsystem;

/**
 * Comando responsável por controlar o motor do Wrist utilizando um controlador PID.
 * Esse comando interage com o {@link PendulumSubsystem} e usa um controlador PID para mover o motor até o ponto de angulação desejado.
 */
public class PendulumCommand extends Command {
  /** Subsystem responsável pelo controle do Wrist */
  private final PendulumSubsystem m_pendulum = RobotContainer.pendulum;
  
  public double var_pendulum = 0;

  /** Creates a new PendulumCommand. */
  public PendulumCommand(double v_pendulum) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    var_pendulum = v_pendulum;
    addRequirements(m_pendulum);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pendulum.runMotor(var_pendulum);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
