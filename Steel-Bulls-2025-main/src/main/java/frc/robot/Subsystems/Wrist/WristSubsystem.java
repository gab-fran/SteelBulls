package frc.robot.Subsystems.Wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.Configs;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * Subsystema responsável pelo controle do motor do Wrist.
 */
public class WristSubsystem extends SubsystemBase {
  /** Motor do Wrist */
  private final SparkMax wristMotor = new SparkMax(WristConstants.wristMotorId, MotorType.kBrushless);
  
  /** Encoder do Wrist */
  private final RelativeEncoder wristEncoder = wristMotor.getEncoder();

  /**
   * Construtor da classe. Configura o motor do Wrist usando as configurações definidas em {@link Configs}.
   */
  public WristSubsystem() {
    wristMotor.configure(Configs.wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Retorna a posição atual do motor do Wrist, em unidades do encoder.
   * 
   * @return A posição do motor.
   */
  public double getMotorPosition() {
    return wristEncoder.getPosition();
  }

  public double getMotorSpeed() {
    return wristMotor.get();
  }

  /**
   * Define a velocidade do motor do Wrist.
   * 
   * @param speed A velocidade do motor. Valores positivos giram o motor em uma direção, valores negativos na direção oposta.
   */
  public void setMotor(double speed) {
    wristMotor.set(speed);
  }

  /**
   * Verifica se o motor do Wrist está na posição desejada.
   * 
   * @param pos A posição alvo que o motor deve atingir.
   * @return true se o motor estiver próximo da posição desejada, dentro de um erro de 1 unidade, caso contrário false.
   */
  public boolean atPosition(double pos) {
    return Math.abs(getMotorPosition() - pos) < 1;
  }

  /**
   * Reseta a posição do motor do Joint para 0.
   */
  public void definePosition() {
    wristEncoder.setPosition(0);
  }

  /**
   * Método chamado periodicamente para atualizar o estado do sistema.
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist/Position", getMotorPosition());
    SmartDashboard.putNumber("Wrist/Setpoint", WristConstants.kWristMotorSetPoint);
    SmartDashboard.putNumber("Wrist/Corrente", wristMotor.getOutputCurrent());
  }
}
