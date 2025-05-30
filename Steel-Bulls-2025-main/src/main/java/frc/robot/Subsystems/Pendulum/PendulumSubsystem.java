package frc.robot.Subsystems.Pendulum;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PendulumConstants;

public class PendulumSubsystem extends SubsystemBase {
  // The pendulum motor
  private final SparkMax pendulumLeaderMotor;
  private SparkMaxConfig globalConfig = new SparkMaxConfig();
  
  private final SparkMax pendulumFollowerMotor;
  private SparkMaxConfig followerConfig = new SparkMaxConfig();

  // The pendulum encoder
  private final RelativeEncoder pendulumLeaderEncoder;
  // private final RelativeEncoder pendulumFollowerEncoder;

  // The pendulum PID controller
  private final SparkClosedLoopController pendulumLeaderPIDController;
  // private final SparkClosedLoopController pendulumFollowerPidController;

  /** Creates a new PendulumSubsytem. */
  public PendulumSubsystem() {
    // Create the pendulum motor
    pendulumLeaderMotor = new SparkMax(PendulumConstants.kPendulumMotorLeftCanId, MotorType.kBrushless);
    pendulumFollowerMotor = new SparkMax(PendulumConstants.kPendulumMotorRightCanId, MotorType.kBrushless);

    // Create the pendulum encoder
    pendulumLeaderEncoder = pendulumLeaderMotor.getEncoder();
    //pendulumRightEncoder = pendulumRightMotor.getEncoder();

    // Create PID controller
    pendulumLeaderPIDController = pendulumLeaderMotor.getClosedLoopController();
    //pendulumRightPIDController = pendulumLeftMotor.getClosedLoopController();

    // Calls motors config
    motorsConfig();    
  }  

  public void motorsConfig() {
    globalConfig
      .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // Configure the pendulum motor
     globalConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40);
      //.inverted(false);

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
      //.follow(pendulumLeaderMotor);

    // Configure the pendulum motor - like burn flash
    pendulumLeaderMotor.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pendulumFollowerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pendulum/Position", getPendulumPosition());
    SmartDashboard.putNumber("Pendulum/Setpoint", PendulumConstants.kPendulumMotorSetPoint);
    SmartDashboard.putNumber("Pendulum/Corrente ID 19", pendulumFollowerMotor.getOutputCurrent());
    SmartDashboard.putNumber("Pendulum/Corrente ID 20", pendulumLeaderMotor.getOutputCurrent());
  }

  /**
   * Set the pendulum motor speed
   * @param PendulumMotorSpeed
   */
  public void runMotor(double targetPosition) {
    pendulumLeaderPIDController.setReference(targetPosition, ControlType.kPosition);
  }

  /**
   * Stop the pendulum motor
   * @
   */
  public void stopPendulum() {
    pendulumLeaderPIDController.setReference(0, ControlType.kPosition);    
  }

  public double getPendulumPosition() {
    return pendulumLeaderEncoder.getPosition();
  }
 
  /**
   * Set the pendulum encoder position
   * @param position
   */
  public void resetPendulumEncoder() {
    pendulumLeaderEncoder.setPosition(0);    
  }
}
