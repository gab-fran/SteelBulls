package frc.robot.Commands.Wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.Subsystems.Wrist.WristSubsystem;
import java.util.function.Supplier;

/**
 * Comando responsável por controlar o motor do Wrist utilizando um controlador PID.
 * Esse comando interage com o {@link wristSubsystem} e usa um controlador PID para mover o motor até o ponto de angulação desejado.
 */
public class WristCommand extends Command {
  /** Subsystem responsável pelo controle do Wrist */
  private final WristSubsystem wristSubsystem;

  /** Controlador PID para controlar a angulação do Wrist */
  private final PIDController pidController;

  /** Função fornecida para determinar o ponto de angulação (setpoint) */
  private final Supplier<Double> setpointFunction;

  /** Ponto de angulação do Wrist */
  private static double wristSetpoint;

  /**
   * Construtor do comando.
   * 
   * @param wristSubsystem O {@link WristSubsystem} que será controlado por este comando.
   * @param setpointFunction Função fornecida para obter o setpoint de angulação do Wrist.
   */
  public WristCommand(WristSubsystem wristSubsystem, Supplier<Double> setpointFunction) {
    this.setpointFunction = setpointFunction;
    this.wristSubsystem = wristSubsystem;
    this.pidController = new PIDController(
      WristConstants.kPIDAngulationMotorKp, 
      WristConstants.kPIDAngulationMotorKi, 
      WristConstants.kPIDAngulationMotorKd
    );

    // Define as dependências do comando
    addRequirements(wristSubsystem);
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
    wristSetpoint = setpointFunction.get();

    // Atualiza o setpoint no controlador PID
    pidController.setSetpoint(wristSetpoint);

    // Calcula a velocidade do motor
    double speed = pidController.calculate(wristSubsystem.getMotorPosition());
    
    if (WristConstants.kWristMotorSetPoint > WristConstants.kWristUpMotorMaxPosition) {
      WristConstants.kWristMotorSetPoint = WristConstants.kWristUpMotorMaxPosition;
    }
    if (WristConstants.kWristMotorSetPoint < WristConstants.kWristUpMotorMinPosition) {
      WristConstants.kWristMotorSetPoint = WristConstants.kWristUpMotorMinPosition;
    }
   
    // Define a velocidade do motor
    SmartDashboard.putNumber("Wrist/Position", wristSubsystem.getMotorPosition());
    SmartDashboard.putNumber("Wrist/Speed", speed);

    wristSubsystem.setMotor(speed);
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
