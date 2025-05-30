// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Outtake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.OuttakeConstants;

/**
 * Subsystema responsável pelo controle do motor do Outtake.
 * Permite controlar a direção do motor e exibir dados sobre o seu estado no SmartDashboard.
 */
public class OuttakeSubsystem extends SubsystemBase {

  /** Motor do outtake */
  private final SparkMax outtakeMotor = new SparkMax(OuttakeConstants.outtakeMotorId, MotorType.kBrushless);
  
  /** Encoder do outtake */
  private final RelativeEncoder outtakeEncoder = outtakeMotor.getEncoder();

  /**
   * Construtor da classe. Configura o motor do Joint usando as configurações definidas em {@link Configs}.
   */
  public OuttakeSubsystem() {
    outtakeMotor.configure(Configs.outtakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Define o estado do motor do Outtake, controlando sua direção.
   * 
   * @param state O estado desejado do motor. Use:
   *              | 1 para girar para frente.
   *              | -1 para girar para trás.
   *              | 0 para parar o motor.
   */
  public void setOuttake(int state) {
    switch (state) {
      case 1: 
        outtakeMotor.set(1); // Aciona o motor do Outtake para girar para frente
        break;
      case -1: 
        outtakeMotor.set(-1); // Aciona o motor do Outtake para girar para trás
        break;
      case 0: 
        outtakeMotor.set(0); // Para o motor do Outtake
        break;
    }
  }

  /**
   * Retorna a posição atual do motor do Joint, em unidades do encoder.
   * 
   * @return A posição do motor.
   */
  public double getMotorPosition() {
    return outtakeEncoder.getPosition();
  }

  public double getMotorSpeed() {
    return outtakeMotor.get();
  }

  /**
   * Define a velocidade do motor do Joint.
   * 
   * @param speed A velocidade do motor. Valores positivos giram o motor em uma direção, valores negativos na direção oposta.
   */
  public void setMotor(double speed) {
    outtakeMotor.set(speed);
  }
  
  /**
   * Método chamado periodicamente para atualizar o estado do sistema.
   * Exibe a corrente atual do motor do Outtake no SmartDashboard.
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Outtake/Position", getMotorPosition());
    SmartDashboard.putNumber("Outtake/Setpoint", OuttakeConstants.kOuttakeMotorSetPoint);
    SmartDashboard.putNumber("Outtake/Corrente", outtakeMotor.getOutputCurrent());
  }
}
