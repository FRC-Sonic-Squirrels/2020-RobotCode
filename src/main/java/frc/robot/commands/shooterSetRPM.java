/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.fearxzombie.limelight;
import com.team2930.lib.util.geometry;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.shooterSubsystem;

public class shooterSetRPM extends CommandBase {
  private shooterSubsystem m_shooter;
  private driveSubsystem m_drive;
  private limelight m_limelight = null;
  private Translation2d m_target;

  /**
   * Creates a new shooterSetRPM.
   */
  public shooterSetRPM(shooterSubsystem shooter, limelight ll, driveSubsystem drive) {
    // TODO: enter actual coordinates of target here
    m_target = new Translation2d(2.0, 0.0);
    m_limelight = ll;
    m_shooter = shooter;
    m_drive = drive;
    // NOTE: we do not set drive as a requirement, we just call get
    addRequirements(shooter);
    // TODO: make limelight a subsystem so we can add it as a requirement?
    // addRequirements(ll);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double targetDistance = -1;

    if (m_limelight != null) {
      // make sure limelight instance is valid
      if (m_limelight.getTV() == 1) {
        // we have limelight target lock
        // TODO: these constants should be in Constants.java
        targetDistance = m_limelight.getDist(0.6096, 2.5019, 32, false);

        // TODO: our own fudge factor for distance, this shouldn't happen in limelight class
        //targetDistance = targetDistance / 1.1154856;
      }
    }

    if (targetDistance < 0) {
      // limelight failed to give a distance, get the distance from odometery
      Pose2d future_pose = m_drive.getFuturePose(0.1);
  
      // estimated distance to target based on odometery
      targetDistance = geometry.distance2Target(future_pose, m_target);  
    }

    if (targetDistance > 0) {
      // set flywheel RPM
      m_shooter.setShooterRPM(m_shooter.getRPMforDistanceMeter(targetDistance));
    }


    // TODO: all this limelight stuf belongs in a limelight subsystem
    // SmartDashboard.putBoolean("1xZoom", oneXZoom);
    // SmartDashboard.putBoolean("2xZoom", twoXZoom);
    // SmartDashboard.putBoolean("3xZoom", threeXZoom);
    // SmartDashboard.putBoolean("LL_TARGETLOCK", lock);
    // if (currentDist >= oneXDist || getTV() == 1){
    // set1xZoom();
    // System.out.println("Switching to 1x Zoom");
    // } else if (currentDist >= twoXDist || getTV() == 1){
    // set2xZoom();
    // System.out.println("Switching to 2x Zoom");
    // } else if (currentDist >= threeXDist || getTV() == 1) {
    // set3xZoom();
    // System.out.println("Switching to 3x Zoom");
    // } if (a2 == 0 || getTV() == 0){
    // dist = -1;
    // set1xZoom();
    // System.out.println("No target in range, switching to normal zoom.");
    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
