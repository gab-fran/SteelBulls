package frc.robot.Commands.Outtake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Outtake.OuttakeSubsystem;

/**
 * Comando responsável por controlar o estado do Outtake.
 * Esse comando interage com o {@link OuttakeSubsystem} para definir o estado desejado do Outtake.
 */
public class ChageSetpointOuttakeCmd extends Command {

  /** Subsystem do Outtake utilizado para realizar as ações */
  private final OuttakeSubsystem outtakeSubsystem;

  /** Estado desejado do Outtake (1 para girar para frente, 0 para parar) */
  private Integer state;

  /**
   * Construtor do comando.
   * 
   * @param outtakeSubsystem O {@link OuttakeSubsystem} que será controlado por este comando.
   * @param state O estado desejado do Outtake. Use 1 para girar para frente, -1 para girar para trás e 0 para parar.
   */
  public ChageSetpointOuttakeCmd(OuttakeSubsystem outtakeSubsystem, Integer state) {
    this.outtakeSubsystem = outtakeSubsystem;
    this.state = state;
  }

  /**
   * Método chamado periodicamente enquanto o comando está sendo executado.
   * Define o estado do Outtake de acordo com o valor de {@link #state}.
   */
  @Override
  public void execute() {
    outtakeSubsystem.setOuttake(state);
  }

  /**
   * Define se o comando foi finalizado.
   * Esse comando termina imediatamente após ser executado uma vez, portanto, sempre retorna true.
   * 
   * @return true, indicando que o comando foi concluído.
   */
  @Override
  public boolean isFinished() {
    return true;
  }
}
