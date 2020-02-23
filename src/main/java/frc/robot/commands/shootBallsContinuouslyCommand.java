/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.fearxzombie.limelight;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.indexerSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.turretSubsystem;
import frc.robot.RobotContainer;

public class shootBallsContinuouslyCommand extends CommandBase {

  indexerSubsystem m_indexer;
  turretSubsystem m_turret;
  shooterSubsystem m_shooter;
  limelight m_limelight;
  driveSubsystem m_drive;

  private double steer_k = 0.1;
  private double tv;
  private double tx;
  private double limelightSteerCommand = 0;

  public shootBallsContinuouslyCommand(indexerSubsystem indexer, turretSubsystem turret, shooterSubsystem shooter, limelight ll_util, driveSubsystem drive) {
    addRequirements(indexer);
    addRequirements(turret);
    addRequirements(shooter);
    addRequirements(drive);
    m_indexer = indexer;
    m_turret = turret;
    m_shooter = shooter;
    m_limelight = ll_util;
    m_drive = drive;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    tv = m_limelight.getTV();
    tx = m_limelight.getTX();

    if (tv != 1) {
      RobotContainer.limelightOnTarget = false;
      limelightSteerCommand = 0;
      m_turret.setPercentOutput(RobotContainer.m_operatorController.getX(Hand.kLeft));
      return;
    }

    double distance = m_limelight.getDist(0.6096, 2.5019, 32);
    m_shooter.setShooterRPM(m_shooter.getRPMforDistanceMeter(distance) + 200);
    SmartDashboard.putNumber("distance", distance);

    limelightSteerCommand = tx * steer_k;
    m_drive.arcadeDrive(0, -limelightSteerCommand * 0.5);

    if (m_shooter.isAtSpeed() == true && Math.abs(m_limelight.getTX()) < 5) {
      RobotContainer.limelightOnTarget = true;
      m_indexer.ejectIndexer();
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    m_indexer.stopIndexer();
    m_shooter.setShooterRPM(0);
    m_turret.stop();
    RobotContainer.limelightOnTarget = false;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
