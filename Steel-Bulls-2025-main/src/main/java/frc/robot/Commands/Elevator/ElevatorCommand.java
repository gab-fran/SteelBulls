package frc.robot.Commands.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import java.util.function.Supplier;

/**
 * Comando responsável por controlar o motor do Elevator utilizando um controlador PID.
 * Esse comando interage com o {@link ElevatorSubsystem} e usa um controlador PID para mover o motor até o ponto de angulação desejado.
 */
public class ElevatorCommand extends Command {
  /** Subsystem responsável pelo controle do Elevator */
  private final ElevatorSubsystem elevatorSubsystem;

  /** Controlador PID para controlar a angulação do Elevator */
  private final PIDController pidController;

  /** Função fornecida para determinar o ponto de angulação (setpoint) */
  private final Supplier<Double> setpointFunction;

  /** Ponto de angulação do Elevator */
  private static double elevatorSetpoint;

  /**
   * Construtor do comando.
   * 
   * @param elevatorSubsystem O {@link ElevatorSubsystem} que será controlado por este comando.
   * @param setpointFunction Função fornecida para obter o setpoint de angulação do Elevator.
   */
  public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, Supplier<Double> setpointFunction) {
    this.setpointFunction = setpointFunction;
    this.elevatorSubsystem = elevatorSubsystem;
    this.pidController = new PIDController(
      ElevatorConstants.kPIDAngulationMotorKp, 
      ElevatorConstants.kPIDAngulationMotorKi, 
      ElevatorConstants.kPIDAngulationMotorKd
    );

    // Define as dependências do comando
    addRequirements(elevatorSubsystem);
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
    elevatorSetpoint = setpointFunction.get();

    // Atualiza o setpoint no controlador PID
    pidController.setSetpoint(elevatorSetpoint);

    // Calcula a velocidade do motor
    double speed = pidController.calculate(elevatorSubsystem.getMotorPosition());

    /*if (ElevatorConstants.kElevatorMotorSetPoint > ElevatorConstants.kElevatorUpMotorMaxPosition) {
      ElevatorConstants.kElevatorMotorSetPoint = ElevatorConstants.kElevatorUpMotorMaxPosition;
    }
    if (ElevatorConstants.kElevatorMotorSetPoint < ElevatorConstants.kElevatorUpMotorMinPosition) {
      ElevatorConstants.kElevatorMotorSetPoint = ElevatorConstants.kElevatorUpMotorMinPosition;
    }*/

    // Define a velocidade do motor
    SmartDashboard.putNumber("Elevator/Position", elevatorSubsystem.getMotorPosition());
    SmartDashboard.putNumber("Elevator/Speed", speed);

    elevatorSubsystem.setMotor(speed);
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
