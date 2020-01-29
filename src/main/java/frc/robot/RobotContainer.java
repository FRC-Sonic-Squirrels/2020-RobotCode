/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.driveConstants;
import frc.robot.commands.driveCommand;
import frc.robot.commands.elevatorCommand;
import frc.robot.commands.indexStage1;
import frc.robot.commands.limelightTurretVisionCommand;
import frc.robot.commands.shooterCommand;
//import frc.robot.commands.shooterHalfCommand;
//import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.elevatorSubsystem;
import frc.robot.subsystems.turretSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.controlPanelSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final static driveSubsystem m_driveSubsystem = new driveSubsystem();
  private final turretSubsystem m_turretSubsystem = new turretSubsystem();
  //private final limelightTurretVisionCommand m_turretVisionCommand = new limelightTurretVisionCommand(m_turretSubsystem);
  //private final driveCommand m_driveCommand = new driveCommand(m_driveSubsystem);
  public static final shooterSubsystem m_shooter = new shooterSubsystem();
  public static final shooterCommand m_shooterCommand = new shooterCommand(m_shooter);
  public static final elevatorSubsystem m_elevatorSubsystem = new elevatorSubsystem();
  public final elevatorCommand m_elevatorCommand = new elevatorCommand(m_elevatorSubsystem);
  private final controlPanelSubsystem m_controlPanelSubsystem = new controlPanelSubsystem();
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public static XboxController m_driveController = new XboxController(driveConstants.k_driveController);
  public static XboxController m_operatorController = new XboxController(driveConstants.k_operatorController);
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_driveSubsystem.setDefaultCommand(new driveCommand(m_driveSubsystem));
    m_turretSubsystem.setDefaultCommand(new limelightTurretVisionCommand(m_turretSubsystem));
    //m_elevatorSubsystem.setDefaultCommand(new elevatorCommand(m_elevatorSubsystem));
    
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final JoystickButton abutton = new JoystickButton(m_driveController, Button.kA.value);
    abutton.toggleWhenPressed(new elevatorCommand(m_elevatorSubsystem));
    final JoystickButton bbutton = new JoystickButton(m_driveController, Button.kBumperRight.value);
    final JoystickButton xbutton = new JoystickButton(m_driveController, Button.kX.value);
    final JoystickButton opBbutton = new JoystickButton(m_operatorController, Button.kBumperRight.value);
    final JoystickButton ybutton = new JoystickButton(m_driveController, Button.kBumperLeft.value);
    bbutton.toggleWhenPressed(new shooterCommand(m_shooter));
    //bbutton.whenReleased(command)
    //xbutton.whenPressed( () -> shooterCommand.index()).whenReleased(() -> IndexerSubsystem.indexLoad.set(ControlMode.PercentOutput, 0));
    ybutton.toggleWhenPressed(new indexStage1());
    ybutton.whenPressed(() -> m_controlPanelSubsystem.setPosition(0), m_controlPanelSubsystem);
    xbutton.whenPressed(() -> m_controlPanelSubsystem.setPosition(1 * 4096), m_controlPanelSubsystem);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //limelightVisionCommand m_autoCommand = new limelightVisionCommand();
    //return m_autoCommand;

 // }
}
