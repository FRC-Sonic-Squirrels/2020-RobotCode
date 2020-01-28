/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.controlPanelConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;


public class controlPanelCommand extends CommandBase {
  /**
   * Creates a new controlPanelCommand.
   */
  private WPI_TalonSRX colorWheelMotor = new WPI_TalonSRX(controlPanelConstants.kMotorPort);

  public controlPanelCommand() {
    colorWheelMotor.configFactoryDefault();
    colorWheelMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, controlPanelConstants.kPIDLoopIdx, controlPanelConstants.kTimeoutMs);
    colorWheelMotor.setSensorPhase(controlPanelConstants.kSensorPhase);
    colorWheelMotor.setInverted(controlPanelConstants.kMotorInvert);
    colorWheelMotor.configNominalOutputForward(0, controlPanelConstants.kTimeoutMs);
    colorWheelMotor.configNominalOutputReverse(0, controlPanelConstants.kTimeoutMs);
    colorWheelMotor.configPeakOutputForward(.2, controlPanelConstants.kTimeoutMs);
    colorWheelMotor.configPeakOutputReverse(-.2, controlPanelConstants.kTimeoutMs);
    colorWheelMotor.configAllowableClosedloopError(0, controlPanelConstants.kPIDLoopIdx, controlPanelConstants.kTimeoutMs);
    colorWheelMotor.config_kF(controlPanelConstants.kPIDLoopIdx, controlPanelConstants.kGains.kF, controlPanelConstants.kTimeoutMs);
    colorWheelMotor.config_kP(controlPanelConstants.kPIDLoopIdx, controlPanelConstants.kGains.kP, controlPanelConstants.kTimeoutMs);
    colorWheelMotor.config_kI(controlPanelConstants.kPIDLoopIdx, controlPanelConstants.kGains.kI, controlPanelConstants.kTimeoutMs);
    colorWheelMotor.config_kD(controlPanelConstants.kPIDLoopIdx, controlPanelConstants.kGains.kD, controlPanelConstants.kTimeoutMs);
    int absolutePosition = colorWheelMotor.getSelectedSensorPosition();
    absolutePosition &= 0xFFF;
    
    if (controlPanelConstants.kSensorPhase) {
      absolutePosition *= -1;
    }
    
    if (controlPanelConstants.kMotorInvert) {
      absolutePosition *= -1;
    }

    colorWheelMotor.setSelectedSensorPosition(0, controlPanelConstants.kPIDLoopIdx, controlPanelConstants.kTimeoutMs);
  }

  public void setSpeed(double speed) {
    colorWheelMotor.set(ControlMode.Velocity, speed);
  }

  public void setPosition(double position) {
    colorWheelMotor.set(ControlMode.Position, position);
  }
    // Use addRequirements() here to declare subsystem dependencies.

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
