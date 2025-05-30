package frc.robot.Subsystems.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Configs;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * Subsystema responsável pelo controle do motor do Elevator.
 */
public class ElevatorSubsystem extends SubsystemBase {
  /** Motor do Elevator */
  private final SparkMax elevatorMotor = new SparkMax(ElevatorConstants.elevatorMotorId, MotorType.kBrushless);
  
  /** Encoder do Elevator */
  private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  /**
   * Construtor da classe. Configura o motor do Elevator usando as configurações definidas em {@link Configs}.
   */
  public ElevatorSubsystem() {
    elevatorMotor.configure(Configs.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Analógico
  public void drive(double speed2, boolean fieldRelative) {
    elevatorMotor.set(speed2);
  }

  /**
   * Retorna a posição atual do motor do Elevator, em unidades do encoder.
   * 
   * @return A posição do motor.
   */
  public double getMotorPosition() {
    return elevatorEncoder.getPosition();
  }

  public double getMotorSpeed() {
    return elevatorMotor.get();
  }

  /**
   * Define a velocidade do motor do Elevator.
   * 
   * @param speed A velocidade do motor. Valores positivos giram o motor em uma direção, valores negativos na direção oposta.
   */
  public void setMotor(double speed) {
    elevatorMotor.set(speed);
  }

  /**
   * Verifica se o motor do Elevator está na posição desejada.
   * 
   * @param pos A posição alvo que o motor deve atingir.
   * @return true se o motor estiver próximo da posição desejada, dentro de um erro de 1 unidade, caso contrário false.
   */
  public boolean atPosition(double pos) {
    return Math.abs(getMotorPosition() - pos) < 1;
  }

  /**
   * Reseta a posição do motor do Elevator para 0.
   */
  public void definePosition() {
    elevatorEncoder.setPosition(0);
  }

  /**
   * Método chamado periodicamente para atualizar o estado do sistema.
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator/Position", getMotorPosition());
    SmartDashboard.putNumber("Elevator/Setpoint", ElevatorConstants.kElevatorMotorSetPoint);
    SmartDashboard.putNumber("Elevator/Corrente", elevatorMotor.getOutputCurrent());
  }
}
