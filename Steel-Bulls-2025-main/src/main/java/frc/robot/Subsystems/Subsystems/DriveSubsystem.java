package frc.robot.Subsystems.Subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // Configuração Pigeon 2.0
  Pigeon2 pidgey = new Pigeon2(12); // ID

  // Sensor
  Rotation2d rotation = pidgey.getRotation2d();

  private Field2d field = new Field2d();

  // Cria o MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
    DriveConstants.frontLeftDrivingCanId,
    DriveConstants.frontLeftTurningCanId,
    DriveConstants.kFrontLeftChassisAngularOffset);
  
  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
    DriveConstants.frontRightDrivingCanId,
    DriveConstants.frontRightTurningCanId,
    DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
    DriveConstants.rearLeftDrivingCanId,
    DriveConstants.rearLeftTurningCanId,
    DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
    DriveConstants.rearRightDrivingCanId,
    DriveConstants.rearRightTurningCanId,
    DriveConstants.kBackRightChassisAngularOffset);

  // Classe de odometria para rastrear a pose do robô
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics, // Configuração cinemática do drive
      rotation,
        new SwerveModulePosition[] { // Posições das rodas
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
    });

  /** Cria um novo DriveSubsystem. */
  public DriveSubsystem() {
    // Relatório de uso para o modelo MAXSwerve
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    try{
      RobotConfig config = RobotConfig.fromGUISettings();
      // Configure AutoBuilder
      AutoBuilder.configure(
        this::getPose, 
        this::resetPose, 
        this::getChassisSpeeds, 
        this::driveRobotRelative, 
        new PPHolonomicDriveController(
          Constants.translationConstants,
          Constants.rotationConstants
        ),
        config,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this
      );
    } catch(Exception e){
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    Rotation2d rotation = pidgey.getRotation2d();
    // Atualiza a odometria no bloco periódico
    m_odometry.update(
      rotation,
        new SwerveModulePosition[] { // Posições das rodas
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });
      field.setRobotPose(getPose());
    }

    public void resetPose(Pose2d pose) {
      System.out.println("Resetting pose to: " + pose);
      // Redefine a pose com a nova posição e rotação
      m_odometry.resetPosition(rotation, 
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
    }
   
    public ChassisSpeeds getChassisSpeeds() {
      final ChassisSpeeds desiredSpeeds = new ChassisSpeeds(DriveConstants.globalxSpeed, DriveConstants.globalySpeed, DriveConstants.globalRot);
      return desiredSpeeds;
    }
     
    // See drive constants for details
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
      setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
    }
 
    /**
     * Retorna a pose estimada atualmente do robô.
     *
     * @return A pose.
    */
    public Pose2d getPose() {
      return m_odometry.getPoseMeters();
    }

    /**
     * Redefine a odometria para a pose especificada.
     *
     * @param pose A pose para a qual definir a odometria.
    */
    public void resetOdometry(Pose2d pose) {
      m_odometry.resetPosition(
        rotation,
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          },
        pose);
  }

  // Modo turbo - Aumentar a velocidade
  public void highVelocity() {
    DriveConstants.kMaxSpeedMetersPerSecond = 4;
    System.out.println("High: " + DriveConstants.kMaxSpeedMetersPerSecond);
  }

  // Modo turbo - Reduzir para a velocidade padrão
  public void lowVelocity() {
    DriveConstants.kMaxSpeedMetersPerSecond = 1;
    System.out.println("Low: " + DriveConstants.kMaxSpeedMetersPerSecond);
  }

  /**
   * Método para dirigir o robô usando informações do joystick.
   *
   * @param xSpeed ​​Velocidade do robô na direção x (para frente).
   * @param ySpeed ​​Velocidade do robô na direção y (para os lados).
   * @param rot Taxa angular do robô.
   * @param fieldRelative Se as velocidades x e y fornecidas são relativas ao
   * campo.
  */
  // Mexe a parte de baixo
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Converta as velocidades comandadas nas unidades corretas para o sistema de transmissão
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
    
    int m_TurboLento = DriveConstants.m_TurboLento_1;
    
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
          pidgey.getRotation2d())
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    if(m_TurboLento > 0)
      // Recebendo valor da velocidade em metros por segundo
      lowVelocity();
    else
      // Recebendo valor da velocidade em metros por segundo
      highVelocity();
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  // Posição diamante
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  }

  public void setFast() {
    DriveConstants.m_TurboLento_1 = 1;
  }

  public void setSlow() {
    DriveConstants.m_TurboLento_1 = 0;
  }
  
  /**
   * Define os estados do módulo de desvio.
   *
   * @param desiredStates Os estados do módulo de desvio desejados.
  */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }
  
  /** Redefine os codificadores de unidade para ler atualmente uma posição de 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }
  
  /** Zera o rumo do robô. */
  public void zeroHeading() {
    pidgey.setYaw(0);
  }

  /**
   * Retorna o rumo do robô.
   *
   * @retorna o rumo do robô em graus, de -180 a 180
  */
  public double getHeading() {
    return pidgey.getRotation2d().getDegrees();
  }

  /**
   * Retorna a taxa de giro do robô.
   *
   * @return A taxa de giro do robô, em graus por segundo
  */
  public double getTurnRate() {
    return pidgey.getRotation2d().getDegrees() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  // Teste do Path Planner - Autônomo
  private void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative)
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
  }

  private void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds, false);
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    };
  }
}
