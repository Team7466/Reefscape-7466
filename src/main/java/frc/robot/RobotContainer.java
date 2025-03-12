// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.Dealgaefier;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final Elevator m_Elevator = new Elevator();
  private final EndEffector m_EndEffector = new EndEffector();
  private final Intake m_Intake = new Intake();
  private final Dealgaefier m_Dealgaefier = new Dealgaefier();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverXbox =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandPS5Controller driverPS =
      new CommandPS5Controller(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController operatorXbox =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private double speed = 1.0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    m_DriveSubsystem.setDefaultCommand(
        m_DriveSubsystem.driveCommand(
            () -> speed * -driverPS.getLeftY(), () -> -driverPS.getRightX()));

    m_Elevator.setDefaultCommand(m_Elevator.run(() -> m_Elevator.elevStop()));
    m_EndEffector.setDefaultCommand(m_EndEffector.run(() -> m_EndEffector.stop()));
    m_Dealgaefier.setDefaultCommand(m_Dealgaefier.run(() -> m_Dealgaefier.stop()));
    m_Intake.setDefaultCommand(m_Intake.run(() -> m_Intake.stop()));
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

    driverPS.R1().whileTrue(Commands.run(() -> speed = 0.6));
    driverPS.R1().onFalse(Commands.run(() -> speed = 1.0));
    driverPS.triangle().whileTrue(m_Elevator.run(() -> m_Elevator.elevVoltage(0.1)));
    operatorXbox.a().onTrue(m_Elevator.run(() -> m_Elevator.elevSet(10.29)));
    operatorXbox.b().whileTrue(m_Intake.run(() -> m_Intake.set(0.5)));
    operatorXbox.y().whileTrue(m_Dealgaefier.run(() -> m_Dealgaefier.set(0.5)));
    operatorXbox.x().whileTrue(m_EndEffector.run(() -> m_EndEffector.set(0.5)));
    operatorXbox.povUp().whileTrue(m_Elevator.run(() -> m_Elevator.elevUp()));
    operatorXbox.povDown().whileTrue(m_Elevator.run(() -> m_Elevator.elevUp()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new WaitCommand(1);
  }
}
