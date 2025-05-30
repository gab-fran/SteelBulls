package frc.robot.Commands.Joints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Joints.JointSubsystem;
import frc.robot.Constants.*;

import java.util.function.Supplier;

/**
 * Comando responsável por controlar o motor do Joint utilizando um controlador PID.
 * Esse comando interage com o {@link JointSubsystem} e usa um controlador PID para mover o motor até o ponto de angulação desejado.
 */
public class JointCommand extends Command {
  /** Subsystem responsável pelo controle do Joint */
  private final JointSubsystem jointSubsystem;

  /** Controlador PID para controlar a angulação do Joint */
  private final PIDController pidController;

  /** Função fornecida para determinar o ponto de angulação (setpoint) */
  private final Supplier<Double> setpointFunction;

  /** Ponto de angulação do Joint */
  private static double jointSetpoint;

  /**
   * Construtor do comando.
   * 
   * @param jointSubsystem O {@link JointSubsystem} que será controlado por este comando.
   * @param setpointFunction Função fornecida para obter o setpoint de angulação do Joint.
   */
  public JointCommand(JointSubsystem jointSubsystem, Supplier<Double> setpointFunction) {
    this.setpointFunction = setpointFunction;
    this.jointSubsystem = jointSubsystem;
    this.pidController = new PIDController(
      JointConstants.kPIDAngulationMotorKp, 
      JointConstants.kPIDAngulationMotorKi, 
      JointConstants.kPIDAngulationMotorKd
    );

    // Define as dependências do comando
    addRequirements(jointSubsystem);
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
    jointSetpoint = setpointFunction.get();

    // Atualiza o setpoint no controlador PID
    pidController.setSetpoint(jointSetpoint);

    // Calcula a velocidade do motor
    double speed = pidController.calculate(jointSubsystem.getMotorPosition());

    if (JointConstants.kJointMotorSetPoint > JointConstants.kJointUpMotorMaxPosition) {
      JointConstants.kJointMotorSetPoint = JointConstants.kJointUpMotorMaxPosition;
    }
    if (JointConstants.kJointMotorSetPoint < JointConstants.kJointUpMotorMinPosition) {
      JointConstants.kJointMotorSetPoint = JointConstants.kJointUpMotorMinPosition;
    }

    // Define a velocidade do motor
    SmartDashboard.putNumber("Joint/Position", jointSubsystem.getMotorPosition());
    SmartDashboard.putNumber("Joint/Speed", speed);

    jointSubsystem.setMotor(speed);
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
