// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SpeakerShoot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.RightShoot;
import frc.robot.commands.LeftShoot;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  private final IntakeCommand intakeCommandD = new IntakeCommand(intake, .2);
  private final SpeakerShoot sShoot = new SpeakerShoot(shooter, intake);

  private final RightShoot rShoot = new RightShoot(shooter, intake);
  private final LeftShoot lShoot = new LeftShoot(shooter, intake);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
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
    driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    driverController.a().toggleOnTrue(intakeCommandD);

    driverController.x().toggleOnTrue(new StartEndCommand(intake :: reverse, intake :: stop));

    driverController.y().toggleOnTrue(sShoot);

    driverController.povDown().toggleOnTrue(new StartEndCommand(climber :: winchRetract, climber :: stopWinch));
    driverController.povUp().toggleOnTrue(new StartEndCommand(climber :: winchReverse, climber :: stopWinch));

    driverController.rightBumper().toggleOnTrue(rShoot);
    driverController.leftBumper().toggleOnTrue(lShoot);
    
    /** 
    driverController.rightBumper().whileTrue(new InstantCommand(() -> climber.setFlipSpeed(0.3)));
    driverController.rightBumper().whileFalse(new InstantCommand(() -> climber.setFlipSpeed(0)));

    driverController.leftBumper().whileTrue(new InstantCommand(() -> climber.setFlipSpeed(-0.3)));
    driverController.leftBumper().whileFalse(new InstantCommand(() -> climber.setFlipSpeed(0)));
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
