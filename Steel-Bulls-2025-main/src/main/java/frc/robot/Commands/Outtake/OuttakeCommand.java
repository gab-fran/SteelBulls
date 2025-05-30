package frc.robot.Commands.Outtake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.Subsystems.Outtake.OuttakeSubsystem;
import java.util.function.Supplier;

/**
 * Comando responsável por controlar o motor do Joint utilizando um controlador PID.
 * Esse comando interage com o {@link outakeSubsystem} e usa um controlador PID para mover o motor até o ponto de angulação desejado.
 */
public class OuttakeCommand extends Command {
  /** Subsystem responsável pelo controle do Outtake */
  private final OuttakeSubsystem outtakeSubsystem;

  /** Controlador PID para controlar a angulação do Outtake */
  private final PIDController pidController;

  /** Função fornecida para determinar o ponto de angulação (setpoint) */
  private final Supplier<Double> setpointFunction;

  /** Ponto de angulação do Outtake */
  private static double outtakeSetpoint;

  /**
   * Construtor do comando.
   * 
   * @param outtakeSubsystem O {@link OuttakeSubsystem} que será controlado por este comando.
   * @param setpointFunction Função fornecida para obter o setpoint de angulação do Outtake.
   */
  public OuttakeCommand(OuttakeSubsystem outtakeSubsystem, Supplier<Double> setpointFunction) {
    this.setpointFunction = setpointFunction;
    this.outtakeSubsystem = outtakeSubsystem;
    this.pidController = new PIDController(
      OuttakeConstants.kPIDAngulationMotorKp, 
      OuttakeConstants.kPIDAngulationMotorKi, 
      OuttakeConstants.kPIDAngulationMotorKd
    );

    // Define as dependências do comando
    addRequirements(outtakeSubsystem);
  }

  /**
   * Método chamado quando o comando é inicializado.
   */
  @Override
  public void initialize() {
    pidController.reset();
  }

  /**
   * Método chamado periodicamente enquanto o comando está sendo executado.
   * Obtém o ponto de angulação desejado, calcula a velocidade do motor com base no controlador PID
   * e ajusta a velocidade do motor de acordo com o setpoint e as velocidades máximas.
   */
  @Override
  public void execute() {
    outtakeSetpoint = setpointFunction.get();

    // Atualiza o setpoint no controlador PID
    pidController.setSetpoint(outtakeSetpoint);

    // Calcula a velocidade do motor
    double speed = pidController.calculate(outtakeSubsystem.getMotorPosition());

    if (OuttakeConstants.kOuttakeMotorSetPoint > OuttakeConstants.kOuttakeUpMotorMaxPosition) {
      OuttakeConstants.kOuttakeMotorSetPoint = OuttakeConstants.kOuttakeUpMotorMaxPosition;
    }
    if (OuttakeConstants.kOuttakeMotorSetPoint < OuttakeConstants.kOuttakeUpMotorMinPosition) {
      OuttakeConstants.kOuttakeMotorSetPoint = OuttakeConstants.kOuttakeUpMotorMinPosition;
    }

    // Define a velocidade do motor
    SmartDashboard.putNumber("Outtake/Position", outtakeSubsystem.getMotorPosition());
    SmartDashboard.putNumber("Outtake/Speed", speed);

    outtakeSubsystem.setMotor(speed);
  }

  /**
   * Define se o comando foi finalizado.
   * O comando termina quando o controlador PID atinge o setpoint desejado.
   * 
   * @return true, indicando que o comando foi concluído quando o setpoint for atingido.
   */
  @Override
  public boolean isFinished() {
    boolean atSetpoint = pidController.atSetpoint();
    SmartDashboard.putBoolean("At Setpoint?", atSetpoint);
    return pidController.atSetpoint();
  }
}
