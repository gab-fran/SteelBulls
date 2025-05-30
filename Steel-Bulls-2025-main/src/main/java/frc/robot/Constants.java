package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

  /**
   * Contém as constantes utilizadas pelo robô para configurações.
   */
  public class Constants {
    /**
     * Constantes relacionadas ao joystick de controle do robô.
     */
    public static class JoystickConstants {
      /** ID do joystick de controle do robô */
      public static int driveJoystickId = 0;
    }

    /**
     * Constantes relacionadas aos motores de movimentação do robô.
     */
    public static final class DriveConstants {
      public static double globalxSpeed;
      public static double globalySpeed;
      public static double globalRot;
    
      /** Parâmetros de condução - Observe que estas não são as velocidades máximas capazes de
       * o robô, mas sim as velocidades máximas permitidas */
      public static double kMaxSpeedMetersPerSecond = 4.5; // 3.5 padrão
      public static int m_TurboLento_1 = 0;

      public static final double kMaxAngularSpeed = 2 * Math.PI; // Radianos por segundo

      // Configuração do chassi
      public static final double kTrackWidth = Units.inchesToMeters(23.5);// Medidas das rodas em relação à base

      // Distância entre os centros das rodas direita e esquerda do robô
      public static final double kWheelBase = Units.inchesToMeters(23.5);// Medidas das rodas em relação à base

      // Distância entre as rodas dianteiras e traseiras do robô
      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

      // Deslocamentos angulares dos módulos em relação ao chassi em radianos
      public static final double kFrontLeftChassisAngularOffset = Math.PI;
      public static final double kFrontRightChassisAngularOffset = 0;
      public static final double kBackLeftChassisAngularOffset = Math.PI;
      public static final double kBackRightChassisAngularOffset = 0;

      // SPARK MAX CAN IDs
      // Direção
      // Relative encoder ou kPrimaryEncoder
      public static final int frontLeftDrivingCanId = 4; // Frente lado esquerdo
      public static final int rearLeftDrivingCanId = 6; // Atrás lado esquerdo
      public static final int frontRightDrivingCanId = 2; // Frente lado direito
      public static final int rearRightDrivingCanId = 8; // Atrás lado direito

      // Girar
      // Abusolute encoder
      public static final int frontLeftTurningCanId = 3; // Frente lado esquerdo
      public static final int rearLeftTurningCanId = 5; // Atrás lado esquerdo
      public static final int frontRightTurningCanId = 1; // Frente lado direito
      public static final int rearRightTurningCanId = 7; // Atrás lado direito

      public static final boolean kGyroReversed = false;
    }

    /**
     * Constantes relacionadas ao motor dos módulos.
     */
    public static final class ModuleConstants {
      /** O módulo MAXSwerve pode ser configurado com uma das três engrenagens de pinhão: 12T,
      13T ou 14T. Isso altera a velocidade de acionamento do módulo (uma engrenagem de pinhão com
      mais dentes resultará em um robô que aciona mais rápido). */
      public static final int kDrivingMotorPinionTeeth = 14; // Verificar os dentes para não ter problema com o Swerve
    
      // Cálculos necessários para fatores de conversão do motor de acionamento e feed forward
      public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
      public static final double kWheelDiameterMeters = 0.0762;
      public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

      /** 45 dentes na engrenagem cônica da roda, 22 dentes na engrenagem reta do primeiro estágio, 15
      dentes no pinhão cônico*/
      public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
      public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)  / kDrivingMotorReduction;
 
      public static final double kDriveDeadband = 0.05;
    }
    
    /**
     * Constantes relacionadas ao motor de outtake.
     */
    public static class OuttakeConstants {
      /** ID do motor de outtake */
      public static int outtakeMotorId = 15;
   
      /** Constantes PID para controle de angulação */
      public static final double kPIDAngulationMotorKp = 0.04;
      public static final double kPIDAngulationMotorKi = 0;
      public static final double kPIDAngulationMotorKd = 0;

      /** Posição de referência para o motor de outtake */
      public static double kOuttakeMotorSetPoint = 0;

      public static final double kOuttakeUpMotorMaxPosition = 1.5;
      public static final double kOuttakeUpMotorMinPosition = 0.5;

      /** Velocidade máxima do motor para mover o outtake para cima */
      public static final double kOuttakeUpMotorMaxSpeed = 0.10;
      /** Velocidade máxima do motor para mover o outtake para baixo */
      public static final double kOuttakeDownMotorMaxSpeed = -0.10;
    }

    /**
     * Constantes relacionadas ao motor de elevator.
     */
    public static class ElevatorConstants {
      /** ID do motor de elevator */
      public static int elevatorMotorId = 18;
  
      /** Constantes PID para controle de angulação */
      public static final double kPIDAngulationMotorKp = 0.04;
      public static final double kPIDAngulationMotorKi = 0;
      public static final double kPIDAngulationMotorKd = 0;

      /** Posição de referência para o motor de elevator */
      public static double kElevatorMotorSetPoint = 0;

      public static final double kElevatorUpMotorMaxPosition = -200;
      public static final double kElevatorUpMotorMinPosition = 1;

      /** Velocidade máxima do motor para mover o elevator para cima */
      public static final double kElevatorUpMotorMaxSpeed = 1;
      /** Velocidade máxima do motor para mover o elevator para baixo */
      public static final double kElevatorDownMotorMaxSpeed = -1;
    }

    // Constante para o motor
    public static final class NeoMotorConstants {
      public static final double kFreeSpeedRpm = 5676;
    }

    /**
     * Constantes relacionadas ao motor de joint.
     */
    public static class JointConstants {
      /** ID do motor de joint */
      public static int jointMotorId = 17;

      /** Constantes PID para controle de angulação */
      public static final double kPIDAngulationMotorKp = 0.13;
      public static final double kPIDAngulationMotorKi = 0;
      public static final double kPIDAngulationMotorKd = 0;

      /** Posição de referência para o motor de joint */
      public static double kJointMotorSetPoint = 0;

      public static final double kJointUpMotorMaxPosition = 0;
      public static final double kJointUpMotorMinPosition = -10;

      /** Velocidade máxima do motor para mover o joint para cima */
      public static final double kJointUpMotorMaxSpeed = 0.05;
      /** Velocidade máxima do motor para mover o joint para baixo */
      public static final double kJointDownMotorMaxSpeed = -0.05;  
    }

    public static class WristConstants {
      /** ID do motor de wrist */
      public static int wristMotorId = 16;

      /** Constantes PID para controle de angulação */
      public static final double kPIDAngulationMotorKp = 0.64;
      public static final double kPIDAngulationMotorKi = 0;
      public static final double kPIDAngulationMotorKd = 0;

      /** Posição de referência para o motor de wrist */
      public static double kWristMotorSetPoint = 0;

      public static final double kWristUpMotorMaxPosition = 1.3;
      public static final double kWristUpMotorMinPosition = 0;

      /** Velocidade máxima do motor para mover o wrist para cima */
      public static final double kWristUpMotorMaxSpeed = 0.7;
      /** Velocidade máxima do motor para mover o wrist para baixo */
      public static final double kWristDownMotorMaxSpeed = -0.7;
    }

    public static class PendulumConstants {
      public static final int kPendulumMotorRightCanId = 19; // Braço direito
      public static final int kPendulumMotorLeftCanId = 20; // Braço esquerdo

      /** Constantes PID para controle de angulação */
      public static final double kPIDAngulationMotorKp = 0.04;
      public static final double kPIDAngulationMotorKi = 0;
      public static final double kPIDAngulationMotorKd = 0;

      /** Posição de referência para o motor de elevator */
      public static double kPendulumMotorSetPoint = 0;

      public static final double kLiftEncoderPositionPIDMinInput = 0;
      public static final double kLiftEncoderPositionPIDMaxInput = 2 * Math.PI;
      public static final double kLiftMinOutput = 0;
      public static final double kLiftMaxOutput = 2 * Math.PI;
    }

    /**
     * Constantes relacionadas ao motor da backpack.
     */
    public static class BackpackConstants {
      /** ID do motor da backpack */
      public static int backpackMotorId = 5;
    }

    public static final PIDConstants translationConstants = new PIDConstants(5, 0, 0);
    public static final PIDConstants rotationConstants = new PIDConstants(5, 0, 0);
}
