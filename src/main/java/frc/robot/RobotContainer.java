/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.AutoConstants.kMaxSpeedMetersPerSecond;
import static frc.robot.Constants.AutoConstants.kRamseteB;
import static frc.robot.Constants.AutoConstants.kRamseteZeta;
import static frc.robot.Constants.driveConstants.kDriveKinematics;
import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.driveConstants;
import frc.robot.commands.driveCommand;
import frc.robot.commands.limelightTurretVisionCommand;
import frc.robot.commands.manualMode;
import frc.robot.commands.shooterCommand;
import frc.robot.commands.turretHomingCommand;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.elevatorSubsystem;
import frc.robot.subsystems.indexerSubsystem;
import frc.robot.subsystems.turretSubsystem;
import frc.robot.subsystems.shooterSubsystem;
//import frc.robot.subsystems.controlPanelSubsystem;

public class RobotContainer {
  @SuppressWarnings("unused")
  // Subsystems
  private final driveSubsystem m_drive = new driveSubsystem();
  private final turretSubsystem m_turretSubsystem = new turretSubsystem();
  private final shooterSubsystem m_shooter = new shooterSubsystem();
  private final indexerSubsystem m_indexer = new indexerSubsystem();
  @SuppressWarnings("unused")
  private final elevatorSubsystem m_elevatorSubsystem = new elevatorSubsystem();
  //private final controlPanelSubsystem m_controlPanelMotors = new controlPanelSubsystem();

  // Commands
  //public static final shooterCommand m_shooterCommand = new shooterCommand(m_shooter, m_indexer);
  //private final limelightTurretVisionCommand m_turretVisionCommand = new limelightTurretVisionCommand(m_turretSubsystem);
  //private final driveCommand m_driveCommand = new driveCommand(m_driveSubsystem);
  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public static XboxController m_driveController = new XboxController(driveConstants.driveController);
  public static XboxController m_operatorController = new XboxController(driveConstants.operatorController);
  public RobotContainer() {
    configureButtonBindings();

    // default command is arcade drive command
    // TODO: un-NERF the drive command.
    m_drive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_drive.arcadeDrive(0.5 *  -m_driveController.getY(GenericHID.Hand.kLeft),
            0.6 * -m_driveController.getX(GenericHID.Hand.kRight)), m_drive));

    //m_turretSubsystem.setDefaultCommand(new limelightTurretVisionCommand(m_turretSubsystem));
  }

  private void configureButtonBindings() {
    final JoystickButton abutton = new JoystickButton(m_driveController, Button.kA.value);
    final JoystickButton bbutton = new JoystickButton(m_driveController, Button.kB.value);
    final JoystickButton xbutton = new JoystickButton(m_driveController, Button.kX.value);
    final JoystickButton ybutton = new JoystickButton(m_driveController, Button.kY.value);
    //bbutton.toggleWhenPressed(new shooterCommand(m_shooter, m_indexer));
    // final JoystickButton opAbutton = new JoystickButton(m_operatorController, Button.kA.value);
    // final JoystickButton opBbutton = new JoystickButton(m_operatorController, Button.kB.value);
    // opAbutton.whenPressed(new manualMode());
    // opBbutton.whenPressed(new turretHomingCommand());
    // ybutton.toggleWhenPressed(new indexStage1Command(m_indexer));
    // ybutton.whenPressed(() -> m_controlPanelMotors.setPosition(0), m_controlPanelMotors);
    // xbutton.whenPressed(() -> m_controlPanelMotors.setPosition(1 * 4096), m_controlPanelMotors);
    //abutton.whenPressed(() -> m_shooter.setShooterRPM(2000));
    //xbutton.whenPressed(() -> m_shooter.setShooterRPM(2500));
    //ybutton.whenPressed(() -> m_shooter.setShooterRPM(3000));
  }
  
  public Command getNoAutonomousCommand() {
    return new RunCommand(() -> m_drive.tankDriveVolts(0, 0));
  }
  
  public Command getAutonomousCommand() {

    // Drive forward 1 meter
    RamseteCommand ramseteCommand = createTrajectoryCommand(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.0, 0.0, new Rotation2d(0)));

    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }

  public RamseteCommand createTrajectoryCommand(Pose2d startPose, List<Translation2d> translationList, Pose2d endPose) {
    DifferentialDriveVoltageConstraint autoVoltageConstraint;
    TrajectoryConfig config;
  
    // Create a voltage constraint to ensure we don't accelerate too fast
    autoVoltageConstraint = new DifferentialDriveVoltageConstraint(m_drive.getFeedforward(), kDriveKinematics, 6);

    // Create config for trajectory
    config = new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

    var initalTime = System.nanoTime();

    // trajectory to follow. All units in meters.
   var trajectory = TrajectoryGenerator.generateTrajectory(
        startPose,
        translationList,
        endPose,
        config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(trajectory, 
            m_drive::getPose,
            new RamseteController(kRamseteB, kRamseteZeta),
            m_drive.getFeedforward(),
            kDriveKinematics,
            m_drive::getWheelSpeeds,
            m_drive.getLeftPidController(),
            m_drive.getRightPidController(),
            m_drive::tankDriveVolts,
            m_drive);

    var dt = (System.nanoTime() - initalTime) / 1E6;
    System.out.println("RamseteCommand generation time: " + dt + "ms");

    // Run path following command, then stop at the end.
    return ramseteCommand;
  }
  
}
