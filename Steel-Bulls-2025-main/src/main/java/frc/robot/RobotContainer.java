package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.JointConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Commands.Elevator.ChangeSetpointElevatorCmd;
import frc.robot.Commands.Elevator.ElevatorCommand;
import frc.robot.Commands.Joints.ChangeSetpointJointCmd;
import frc.robot.Commands.Joints.JointCommand;
import frc.robot.Commands.Outtake.ChageSetpointOuttakeCmd;
import frc.robot.Commands.Outtake.OuttakeCommand;
import frc.robot.Commands.Pendulum.PendulumCommand;
import frc.robot.Commands.Wrist.WristCommand;
import frc.robot.Commands.Wrist.ChangeSetpointWristCmd;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Joints.JointSubsystem;
import frc.robot.Subsystems.Outtake.OuttakeSubsystem;
import frc.robot.Subsystems.Pendulum.PendulumSubsystem;
import frc.robot.Subsystems.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.Wrist.WristSubsystem;

/**
 * Classe principal para a configuração do robô.
 * Esta classe inicializa e gerencia os subsistemas, controladores e comandos do robô.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  /** Controlador principal do robô baseado no Xbox. */
  private final CommandXboxController driverJoystick_1 = new CommandXboxController(0);
  private final CommandXboxController driverJoystick_2 = new CommandXboxController(1);

  public static JointSubsystem joint = new JointSubsystem();
  public static WristSubsystem wrist = new WristSubsystem();
  public static OuttakeSubsystem outtake = new OuttakeSubsystem();
  public static ElevatorSubsystem elevator = new ElevatorSubsystem();
  public static DriveSubsystem robot = new DriveSubsystem();
  public static PendulumSubsystem pendulum = new PendulumSubsystem();

  private final ChangeSetpointJointCmd changeSetpointJointCmd = new ChangeSetpointJointCmd(0);
  private final ChangeSetpointWristCmd changeSetpointWristCmd = new ChangeSetpointWristCmd(0);
  private final ChangeSetpointElevatorCmd changeSetpointElevatorCmd = new ChangeSetpointElevatorCmd(0);
  private final ChageSetpointOuttakeCmd changeSetpointOuttakeCmd = new ChageSetpointOuttakeCmd(outtake, 0);


  JointCommand jointCommand = new JointCommand(joint, ()-> JointConstants.kJointMotorSetPoint);
  WristCommand wristCommand = new WristCommand(wrist, ()-> WristConstants.kWristMotorSetPoint);
  ElevatorCommand elevatorCommand = new ElevatorCommand(elevator, ()-> ElevatorConstants.kElevatorMotorSetPoint);
  OuttakeCommand outtakeCommand = new OuttakeCommand(outtake, ()-> OuttakeConstants.kOuttakeMotorSetPoint);

  /**
   * Construtor da classe RobotContainer.
   * Inicializa os componentes do robô e configura as associações de comandos aos controles.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("Pendulum - L1", new PendulumCommand(-0.8));
    NamedCommands.registerCommand("Pendulum - L3", new PendulumCommand(-155));
    NamedCommands.registerCommand("Pendulum - Reto", new PendulumCommand(0));
    NamedCommands.registerCommand("Joint - L1", new ChangeSetpointJointCmd(-5.3));
    NamedCommands.registerCommand("Joint - Human Player", new ChangeSetpointJointCmd(-7));
    NamedCommands.registerCommand("Outtake Coral - Soltar", new ChageSetpointOuttakeCmd(outtake, 1));
    NamedCommands.registerCommand("Outtake Coral - Parar", new ChageSetpointOuttakeCmd(outtake, 0));
    NamedCommands.registerCommand("Outtake Coral - Puxar", new ChageSetpointOuttakeCmd(outtake, -1));

    configureBindings();

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above
    SmartDashboard.putData("Sair da zona azul - E", new PathPlannerAuto("GoOutBlueLeft"));
    SmartDashboard.putData("Sair da zona vermelha - E", new PathPlannerAuto("GoOutRedLeft"));
    SmartDashboard.putData("Sair da zona azul - D", new PathPlannerAuto("GoOutBlueRight"));
    SmartDashboard.putData("Sair da zona vermelha - D", new PathPlannerAuto("GoOutRedRight"));
    SmartDashboard.putData("Sair da zona azul Coral - C", new PathPlannerAuto("GoOutBlueCenterCoral"));
    SmartDashboard.putData("Sair da zona vermelha Coral - C", new PathPlannerAuto("GoOutRedCenterCoral"));
  }

  /**
   * Configura os botões do controle para executar comandos específicos quando pressionados.
   */
  private void configureBindings() {
    /*
     * Controle 1
     */
    // Definição dos botões do controle 1
    Trigger r1Trigger = driverJoystick_1.leftBumper(); // Outtake
    Trigger r2Trigger = driverJoystick_1.rightBumper(); // Outtake
    
    // Trigger a1Trigger = driverJoystick_1.rightTrigger(); // Alinhamento das rodas

    Trigger v1Trigger = driverJoystick_1.x(); // Velocity
    Trigger v2Trigger = driverJoystick_1.b(); // Velocity

    // Analógico do andar
    robot.setDefaultCommand(
      new RunCommand(
        () -> robot.drive(
          -MathUtil.applyDeadband(driverJoystick_1.getLeftY(), ModuleConstants.kDriveDeadband),
          -MathUtil.applyDeadband(driverJoystick_1.getLeftX(), ModuleConstants.kDriveDeadband),
          -MathUtil.applyDeadband(driverJoystick_1.getRightX(), ModuleConstants.kDriveDeadband),
          true),
        robot));

    /*a1Trigger
      .and(DriverStation::isEnabled)
      .whileTrue(new RunCommand(
        () -> robot.setX()), 
      robot);*/      

    // Outtake
    r1Trigger
      .and(DriverStation::isEnabled) // Puxa o coral
      .whileTrue(
        new ChageSetpointOuttakeCmd(outtake, -1) // Setpoint de -1 para girar para trás
    );

    // Outtake
    r1Trigger
      .and(DriverStation::isEnabled)
      .whileFalse(
        new ChageSetpointOuttakeCmd(outtake, 0) // Setpoint de 0 para parar
    );

    // Outtake
    r2Trigger
      .and(DriverStation::isEnabled) // Solta o coral
      .whileTrue(
        new ChageSetpointOuttakeCmd(outtake, 1) // Setpoint de 1 para girar para frente
    );

    // Outtake
    r2Trigger
      .and(DriverStation::isEnabled)
      .whileFalse(
        new ChageSetpointOuttakeCmd(outtake, 0) // Setpoint de 0 para parar
    );

    // Velocity
    v1Trigger
      .and(DriverStation::isEnabled)
      .whileTrue(new RunCommand(
        () -> robot.setSlow(), // Reduzir a velocidade
      robot));

    // Velocity
    v2Trigger
      .and(DriverStation::isEnabled)
      .whileTrue(new RunCommand(
        () -> robot.setFast(), // Aumentar a velocidade
      robot));

    /*
     * Controle 2
     */
    // Definição dos botões do controle 2
    Trigger j1Trigger = driverJoystick_2.leftBumper(); // Joint
    Trigger j2Trigger = driverJoystick_2.rightBumper(); // Joint

    /*Trigger w1Trigger = driverJoystick_2.rightBumper(); // Wrist
    Trigger w2Trigger = driverJoystick_2.rightTrigger(); // Wrist*/

    Trigger e1Trigger = driverJoystick_2.povUp(); // Elevator - L3
    Trigger e2Trigger = driverJoystick_2.povDown(); // Elevator - L2 e L1
    //Trigger e4Trigger = driverJoystick_2.povRight(); // Elevator - L4*/
    Trigger e3Trigger = driverJoystick_2.povLeft(); // Elevator - Pegar no chão*/
    
    Trigger p1Trigger = driverJoystick_2.y(); // Pendulum - Mais para frente
    Trigger p2Trigger = driverJoystick_2.x(); // Pendulum - Meio
    Trigger p3Trigger = driverJoystick_2.a(); // Pendulum - Frente para pegar a bola

    Trigger home = driverJoystick_2.start();   

    // Elevator
    e1Trigger
      .and(DriverStation::isEnabled)
      .onTrue(
        new ChangeSetpointElevatorCmd(-155) // Setpoint posição ideal para o L3 - Só pode ser negativo
      );

    // Elevator
    e2Trigger // Posição normal
      .and(DriverStation::isEnabled)
      .onTrue(
        new ChangeSetpointElevatorCmd(3) // Setpoint posição ideal - Início
      );   

    // Elevator
    e3Trigger // Pegar do chão
      .and(DriverStation::isEnabled)
      .onTrue(
        new ChangeSetpointElevatorCmd(-45) // Setpoint posição ideal para pegar o do chão - Só pode ser negativo
      );
    
    // Elevator
    /*e4Trigger
      .and(DriverStation::isEnabled)
      .onTrue(
        new ChangeSetpointElevatorCmd(-348) // Setpoint posição ideal para o L4 - Só pode ser negativo
      );*/

    // Joint2
    j1Trigger
      .and(DriverStation::isEnabled)
      .whileTrue(
        new ChangeSetpointJointCmd(-8.5) // Setpoint posição ideal para o ângulo da garra no human player - Só pode ser negativo
    );

    // Joint
    j2Trigger
      .and(DriverStation::isEnabled)
      .whileTrue(
        new ChangeSetpointJointCmd(-5.3) // Setpoint posição ideal para o ângulo da garra no recife - Só pode ser negativo - L1
    );

    // Wrist
    /* w1Trigger
      .and(DriverStation::isEnabled)
      .whileTrue(
        new ChangeSetpointWristCmd(1.3) // Setpoint posição ideal
    );

    // Wrist
    w1Trigger
      .and(DriverStation::isEnabled)
      .whileFalse(
        new ChangeSetpointWristCmd(0) // Setpoint posição ideal - Só pode ser negativo
    ); */

    // Pendulum
    p1Trigger
      .and(DriverStation::isEnabled)
      .whileTrue(
        new PendulumCommand(-0.8) // Pêndulo para o frente
    );

    // Pendulum
    p2Trigger
      .and(DriverStation::isEnabled)
      .whileTrue(
        new PendulumCommand(0) // Pêndulo para o meio
    );

    // Pendulum
    p3Trigger
      .and(DriverStation::isEnabled)
      .whileTrue(
        new PendulumCommand(-2.2) // Pêndulo para chão para pegar a coral
    );

    // Voltar ao início
    home
      .and(DriverStation::isEnabled)
      .whileTrue(
        new ChangeSetpointJointCmd(0) // Setpoint posição ideal
    );

    joint.setDefaultCommand(new JointCommand(joint, () -> Constants.JointConstants.kJointMotorSetPoint));
    wrist.setDefaultCommand(new WristCommand(wrist, () -> Constants.WristConstants.kWristMotorSetPoint));
    elevator.setDefaultCommand(new ElevatorCommand(elevator, () -> Constants.ElevatorConstants.kElevatorMotorSetPoint));
  }

  /**
   * Obtém o comando autônomo do robô.
   * 
   * @return Comando a ser executado durante o modo autônomo.
   */
  public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return autoChooser.getSelected();
  }
}
