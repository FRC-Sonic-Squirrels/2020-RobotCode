/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.controlPanelConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class controlPanelSubsystem extends SubsystemBase {
  /**
   * Creates a new controlPanelSubsystem.
   */
  private WPI_TalonFX m_motor;

  public controlPanelSubsystem() {
    m_motor = new WPI_TalonFX(controlPanelConstants.kMotorPort);
    WPI_TalonFX m_motor = new WPI_TalonFX(4);
   
    /* Factory Default all hardware to prevent unexpected behaviour */
    m_motor.configFactoryDefault();

    /* Config the sensor used for Primary PID and sensor direction */
    m_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, controlPanelConstants.kPIDLoopIdx,
        controlPanelConstants.kTimeoutMs);

    /* Ensure sensor is positive when output is positive */
    m_motor.setSensorPhase(controlPanelConstants.kSensorPhase);

    /**
     * Set based on what direction you want forward/positive to be. This does not
     * affect sensor phase.
     */
    m_motor.setInverted(controlPanelConstants.kMotorInvert);

    /* Config the peak and nominal outputs, 12V means full */
    m_motor.configNominalOutputForward(0, controlPanelConstants.kTimeoutMs);
    m_motor.configNominalOutputReverse(0, controlPanelConstants.kTimeoutMs);
    m_motor.configPeakOutputForward(.2, controlPanelConstants.kTimeoutMs);
    m_motor.configPeakOutputReverse(-.2, controlPanelConstants.kTimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral
     * within this range. See Table in Section 17.2.1 for native units per rotation.
     */
    m_motor.configAllowableClosedloopError(0, controlPanelConstants.kPIDLoopIdx, controlPanelConstants.kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    m_motor.config_kF(controlPanelConstants.kPIDLoopIdx, controlPanelConstants.kGains.kF,
        controlPanelConstants.kTimeoutMs);
    m_motor.config_kP(controlPanelConstants.kPIDLoopIdx, controlPanelConstants.kGains.kP,
        controlPanelConstants.kTimeoutMs);
    m_motor.config_kI(controlPanelConstants.kPIDLoopIdx, controlPanelConstants.kGains.kI,
        controlPanelConstants.kTimeoutMs);
    m_motor.config_kD(controlPanelConstants.kPIDLoopIdx, controlPanelConstants.kGains.kD,
        controlPanelConstants.kTimeoutMs);

    /**
     * Grab the 360 degree position of the MagEncoder's absolute position, and
     * intitally set the relative sensor to match.
     */
    int absolutePosition = (int) m_motor.getSensorCollection().getIntegratedSensorAbsolutePosition();

    /* Mask out overflows, keep bottom 12 bits */
    absolutePosition &= 0xFFF;
    if (controlPanelConstants.kSensorPhase) {
      absolutePosition *= -1;
    }
    if (controlPanelConstants.kMotorInvert) {
      absolutePosition *= -1;
    }

    /* Set the quadrature (relative) sensor to match absolute */
    m_motor.setSelectedSensorPosition(0, controlPanelConstants.kPIDLoopIdx,
        controlPanelConstants.kTimeoutMs);
  }

  public void setSpeed(double speed) {
    m_motor.set(ControlMode.Velocity, speed);
  }

  public void setPosition(double position) {
    m_motor.set(ControlMode.Position, position);
  }

}