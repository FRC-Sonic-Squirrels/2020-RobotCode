/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.blinkinSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class blinkinLimelightCommand extends CommandBase {
  /**
   * Creates a new blinkinLimelightCommand.
   */

  blinkinSubsystem m_blinkin;

  public blinkinLimelightCommand(blinkinSubsystem subsystem) {
    addRequirements(subsystem);
    m_blinkin = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /* limelight network table entries used to determine LED colors */
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-one");
    double tv = table.getEntry("tv").getDouble(0.0);
    double tx = table.getEntry("tx").getDouble(0.0);
     
    /* Limelight Valid target */
    if (tv == 1.0) {

      /* On target & ready to shoot */
      if (tx >= -1 && tx <= 1.0) {
        m_blinkin.solid_blue();
      } 

      /* Close to target but not in shooting range */
      else {
        m_blinkin.solid_blue_aqua();
      }
    } 

    /* Idle color */
    else {
      m_blinkin.solid_orange();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
