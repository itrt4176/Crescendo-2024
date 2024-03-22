// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.HomeFlipper;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetClimberFlipper;
import frc.robot.commands.Shoot;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "neo"));

  private final VisionSubsystem vision = VisionSubsystem.getInstance();

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();
  private final ShooterSubsystem shooter = new ShooterSubsystem();


  private final IntakeCommand intakeCommandD = new IntakeCommand(intake, Constants.IntakeConstants.INTAKE_SPEED);
  private final Command sShoot = new Shoot(shooter, intake, Constants.ShooterConstants.SPEAKER_SHOT_SPEED)
    .andThen(new WaitCommand(0.3))
    .andThen(new InstantCommand(() -> shooter.setShootSpeed(0), shooter))
    .andThen(new InstantCommand(() -> intake.setIntakeSpeed(0), intake));


  private final Command aShoot = new Shoot(shooter, intake, Constants.ShooterConstants.AMP_SHOT_SPEED)
    .andThen(new WaitCommand(0.5))
    .andThen(new InstantCommand(() -> shooter.setShootSpeed(0), shooter))
    .andThen(new InstantCommand(() -> intake.setIntakeSpeed(0), shooter));

  private final Command tShootTest = new Shoot(shooter, intake, Constants.ShooterConstants.TRAP_TEST_SPEED)
    .andThen(new WaitCommand(0.5))
    .andThen(new InstantCommand(() -> shooter.setShootSpeed(0), shooter))
    .andThen(new InstantCommand(() -> intake.setIntakeSpeed(0), shooter));

  private final HomeFlipper home = new HomeFlipper(climber); // used in sequential command 
  private final HomeFlipper homeReset = new HomeFlipper(climber); //for reseting zero in case

  
  private final SetClimberFlipper flipperToAmp = new SetClimberFlipper(climber, 162);

  private final SequentialCommandGroup ampRoutine = new SequentialCommandGroup(aShoot, home);

  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  private final CommandXboxController testDriveController =
      new CommandXboxController(1);

  private final AbsoluteDrive tuningDriveCommandForward;

  private final AbsoluteDrive tuningDriveCommand90;

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    XboxController driverXbox = new XboxController(0);

   // AbsoluteDriveAdv closAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
     //                                                           () ->  MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_DEADBAND_Y),
     //                                                            null, null, null, null, null, null);

    // FIELD ORIENTED REQUIRES BOTH CONTROLLER X-AXES TO BE INVERTED!
    Command joystickDrive = drivebase.driveCommand(
      () -> applyAllianceInversion( MathUtil.applyDeadband(driverXbox.getLeftY(),
                                  OperatorConstants.LEFT_DEADBAND_Y)),
      () -> applyAllianceInversion(MathUtil.applyDeadband(driverXbox.getLeftX(),
                                  OperatorConstants.LEFT_DEADBAND_X)),
      () -> MathUtil.applyDeadband(-driverXbox.getRightX(),
                                  OperatorConstants.RIGHT_DEADBAND_X)
    );

    tuningDriveCommandForward = new AbsoluteDrive(
      drivebase, //swerve
      () -> 0.5, //vX
      () -> 0.0, //vY
      () -> 0.0, //headingHorizontal
      () -> 0.0 //headingVertical
    );

    tuningDriveCommand90 = new AbsoluteDrive(
      drivebase,
      () -> 0.0,
      () -> 0.5,
      () -> 0.0,
      () -> 0.0

    );

    NamedCommands.registerCommand(
      "speakerShoot",
      new Shoot(shooter, intake, Constants.ShooterConstants.SPEAKER_SHOT_SPEED)
        .andThen(new WaitCommand(0.25))
        .andThen(new InstantCommand(() -> shooter.setShootSpeed(0), shooter))
        .andThen(new InstantCommand(() -> intake.setIntakeSpeed(0), shooter))
    );

    NamedCommands.registerCommand(
      "intake",
      new IntakeCommand(intake, -.22)
    );

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
   /* Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_DEADBAND_Y),
        () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_DEADBAND_X),
        () -> driverController.getRawAxis(2));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_DEADBAND_Y),
        () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_DEADBAND_X),
        () -> driverController.getRawAxis(2));
*/
  drivebase.setDefaultCommand(joystickDrive);
  drivebase.registerVisionPoseCallback(vision::getLimelightEstimatedPose);
   //  drivebase.setDefaultCommand(  !RobotBase.isSimulation() driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
    
  if (RobotBase.isSimulation()) {
    vision.setSimPoseSupplier(drivebase::getPose);
  }
  
  configureBindings();

  autoChooser = drivebase.getAutoChooser();

  SmartDashboard.putData("Auto", autoChooser);
  }
  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    //driverController.a().toggleOnTrue(new StartEndCommand(intake :: lookClimber, intake :: lookIntake));
    
    driverController.leftTrigger().toggleOnTrue(new StartEndCommand(intake :: lookClimber, intake :: lookIntake));

    driverController.a().toggleOnTrue(intakeCommandD);

    driverController.x().toggleOnTrue(new StartEndCommand(intake :: reverse, intake::stop));

    // driverController.y().toggleOnTrue(sShoot);
    driverController.y().toggleOnTrue(tShootTest);

    driverController.b().onTrue(ampRoutine);

    driverController.povUp().onTrue(flipperToAmp);

    driverController.povDown().onTrue(homeReset);

    //driverController.povDown().toggleOnTrue(new StartEndCommand(climber :: winchRetract, climber :: stopWinch));
    //driverController.povUp().toggleOnTrue(new StartEndCommand(climber :: winchReverse, climber :: stopWinch));


    driverController.rightBumper().whileTrue(new InstantCommand(() -> climber.setFlipSpeed(0.3)));
    driverController.rightBumper().whileFalse(new InstantCommand(() -> climber.setFlipSpeed(0)));

    driverController.leftBumper().whileTrue(new InstantCommand(() -> climber.setFlipSpeed(-0.3)));
    driverController.leftBumper().whileFalse(new InstantCommand(() -> climber.setFlipSpeed(0)));
    
    
    // testDriveController.a().onTrue(tuningDriveCommandForward);
    // testDriveController.y().onTrue(tuningDriveCommand90);

    // testDriveController.y().toggleOnTrue(new FunctionalCommand(
    //   () -> drivebase.setModulesToAngle(Rotation2d.fromDegrees(0)),
    //   () -> {},
    //   (interrupted) -> {},
    //   () -> false,
    //   drivebase
    // ));

    // testDriveController.b().toggleOnTrue(new FunctionalCommand(
    //   () -> drivebase.setModulesToAngle(Rotation2d.fromDegrees(90)),
    //   () -> {},
    //   (interrupted) -> {},
    //   () -> false,
    //   drivebase
    // ));

    // testDriveController.a().toggleOnTrue(new FunctionalCommand(
    //   () -> drivebase.setModulesToAngle(Rotation2d.fromDegrees(180)),
    //   () -> {},
    //   (interrupted) -> {},
    //   () -> false,
    //   drivebase
    // ));

    // testDriveController.x().toggleOnTrue(new FunctionalCommand(
    //   () -> drivebase.setModulesToAngle(Rotation2d.fromDegrees(270)),
    //   () -> {},
    //   (interrupted) -> {},
    //   () -> false,
    //   drivebase
    // ));

    

    SmartDashboard.putData("SysId Drive", drivebase.sysIdDriveMotorCommand());
    SmartDashboard.putData("SysId Angle", drivebase.sysIdAngleMotorCommand());
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */ 
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous

    return autoChooser.getSelected();
  }

  private static double applyAllianceInversion(double joystickInput) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return (alliance.get() == Alliance.Red) ? 1.0 * joystickInput : -1.0 * joystickInput;
    } else {
      return -1.0 * joystickInput;
    }
  }
}

