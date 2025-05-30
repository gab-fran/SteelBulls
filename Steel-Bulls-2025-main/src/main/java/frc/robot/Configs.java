package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.ModuleConstants;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

  /**
   * Contém as configurações para os motores do robô.
   */
  public final class Configs {
    /** Configuração para o motor de outtake. */
    public static final SparkMaxConfig outtakeConfig = new SparkMaxConfig();

    /** Configuração para o motor de wrist. */
    public static final SparkMaxConfig wristConfig = new SparkMaxConfig();
    
    /** Configuração para o motor de joint. */
    public static final SparkMaxConfig jointConfig = new SparkMaxConfig();
    
    /** Configuração para o motor de elevator. */
    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    
    /** Configuração para o motor de driving. */
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();

    /** Configuração para o motor de turning. */
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    /** Configuração para o motor de turning. */
    public static final SparkMaxConfig followerConfig = new SparkMaxConfig();

    /** Configuração para o motor de turning. */
    public static final SparkMaxConfig globalConfig = new SparkMaxConfig();

    static {        
      double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;
      double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;


      globalConfig
      .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

      // Configure the pendulum motor
      globalConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40);

      // Configure the pendulum encoder 
      globalConfig
        .encoder
          .positionConversionFactor(Math.PI * 0.01)
          .velocityConversionFactor(1);

      // Configure the pendulum PID controller
      globalConfig
        .closedLoop
          .pidf(0.8, 0, 0, 0);

      followerConfig
        .apply(globalConfig)
        .inverted(true)
        .follow(Constants.PendulumConstants.kPendulumMotorLeftCanId, true);

      // Configuração do motor de outtake
      outtakeConfig
        .idleMode(IdleMode.kBrake) // Modo de inatividade: Brake
        .smartCurrentLimit(40) // Limite de corrente inteligente: 40A
        .inverted(true);
    
      // Configuração do motor de wrist
      wristConfig
        .idleMode(IdleMode.kBrake) // Modo de inatividade: Brake
        .smartCurrentLimit(40) // Limite de corrente inteligente: 40A
        .inverted(true);
      wristConfig
        .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pid(0.04, 0, 0)
          .outputRange(-1, 1);
    
      // Configuração do motor de joint
      jointConfig
        .idleMode(IdleMode.kBrake) // Modo de inatividade: Brake
        .smartCurrentLimit(60) // Limite de corrente inteligente: 60A
        .inverted(false);

      // Configuração do motor de elevator
      elevatorConfig
        .idleMode(IdleMode.kBrake) // Modo de inatividade: Brake
        .smartCurrentLimit(80) // Limite de corrente inteligente: 80A
        .inverted(true);
      elevatorConfig
        .limitSwitch
          .reverseLimitSwitchEnabled(true)
          .reverseLimitSwitchType(Type.kNormallyOpen);
      elevatorConfig
        .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(0.08, 0, 0)
          .outputRange(-1, 1)
        .maxMotion
          // Set MAXMotion parameters for position control
          // Define os parâmetros MAXMotion para controle de posição
          .maxVelocity(6000) // 4200
          .maxAcceleration(6000)
          .allowedClosedLoopError(0.5);
        
      // Configuração do motor driving
      drivingConfig
        .idleMode(IdleMode.kBrake) // Modo de inatividade: Brake
        .smartCurrentLimit(40); // Limite de corrente inteligente: 50A
      drivingConfig
        .encoder
          .positionConversionFactor(drivingFactor) // Metros
          .velocityConversionFactor(drivingFactor / 60.0); // Metros por segundos
      drivingConfig
        .closedLoop
          .velocityFF(drivingVelocityFeedForward)
          .outputRange(-1, 1)
          .pid(0.04, 0, 0)
          .velocityFF(drivingVelocityFeedForward)
          .outputRange(-1, 1);
        
      // Configuração do motor turning
      turningConfig
        .idleMode(IdleMode.kBrake) // Modo de inatividade: Brake
        .smartCurrentLimit(40); // Limite de corrente inteligente: 50A
      turningConfig        
        .absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(true)
          .positionConversionFactor(turningFactor) // Radianos
          .velocityConversionFactor(turningFactor / 60.0); // Radianos por segundos
      turningConfig
        .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
      }
}
