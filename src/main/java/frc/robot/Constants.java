/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public final class Constants {
    public static final class driveConstants {
        public static final int falcon1_leftLead = 12;
        public static final int falcon2_leftFollow = 12;
        public static final int falcon3_rightLead = 14;
        public static final int falcon4_rightFollow = 15;
        public static final int driveTimeout = 30;
        public static final int pigeonCANid = 15;
        public static final int driveController = 0;
        public static final int operatorController = 1;

        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = false;

        public static final boolean kGyroReversed = true;

        // Comp bot track width (center of wheel to center of wheel) is 0.627m
        public static final double kTrackwidthMeters = 0.627;
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackwidthMeters);

        // Determined using frc-characterization tool
        public static final double ksVolts = 0.05; // 0.0491;
        public static final double kvVoltSecondsPerMeter = 2.36; // 2.36;
        public static final double kaVoltSecondsSquaredPerMeter = 0.127;

        // Determined using frc-characterization
        public static final double kPDriveVel = 5.0; // frc-characterization 14.4
        public static final double kDDriveVel = 0.0; // frc-characterization 5.16

        // TalonFX encoders have 2048, Rev Robitics have 4096
        public static final int kEncoderCPR = 2048;

        // Aprox 6 inch (0.1524 meters) traction wheels, measured 0.15836 m
        // Measured circumference = 0.509 m
        public static final double kDistancePerWheelRevolutionMeters = 0.509;
        public static final double kWheelDiameterMeters =
                kDistancePerWheelRevolutionMeters / Math.PI;

        // gear reduction from Falcon Gearbox:
        // Two stages 11:60 then 16:31 for a total gear reduction of 11:120
        public static final double kGearReduction = 11.0 / 120.0;

        // Assumes the encoders are directly mounted on the motor shafts
        public static final double kEncoderDistancePerPulseMeters =
                (kDistancePerWheelRevolutionMeters * kGearReduction) / (double) kEncoderCPR;
    }
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = .5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.25;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
    public static final class turretConstants {
        //TODO get the real turret motor ID
        public static final int turret = 99;
        public static final int kSoftMaxTurretAngle = 90;
        public static final int kSoftMinTurretAngle = -90;
        public static final int kEncoderCPR = 4096; // CTRE Quadrature?

        // BAG motor controlled by Talon
        // Turret inner teeth = 264. * 360
        // gear teeth = 20
        // gearbox = 35:1
        // Total motor to turret rotation ration: 1/396
        // public static final double kGearRation = 1 / 396.0; // turret rotations per motor
        // rotation
        // POST gearbox encoder ratio is:
        // 20:264 or 1/13.2
        public static final double kGearRation = 1 / 13.2; // turret rotations per encoder rotation
        public static final double kTurretRotationsPerTick = kGearRation / kEncoderCPR;
        public static final double kDegreesPerTick = 360 * kTurretRotationsPerTick;
        // TODO: test and increase max velocity and acceleration
        // Max velocity: 90 deg/s
        // Max acceleration: 45 deg/s^2
        public static final double kMaxDegreesPerSecond = 90;
        public static final double kMaxDegreesPerSecondSquared = 45;
        public static final int kTimeout = 30; // Talon command timeout
        public static final int kIndex = 0; // Talon PID index
    }
    public static final class shooterConstants {
        public static final int shooter1 = 4;
        public static final int shooter2 = 5;
        public static final int shooterTimeout = 30;
        public static final int shooterSlotIdx = 0;
        public static final int shooterHood = 1;
    }
    public static final class elevatorConstants {
        public static final int solenoid1 = 6;
        public static final int solenoid2 = 7;
        public static final int elevatorWinch = 12;
        public static final int brakeSolenoid = 14;
        public static final int elevatorPivotTimeout = 30;
        public static final int elevatorSlotIdx = 1;
    }
    public static final class indexConstants {
        public static final int indexIntake = 8;
        public static final int indexBelts = 10;
        public static final int indexKicker = 11;
    }
    public static final class controlPanelConstants {
        public static final int motor = 13;
        public static final int controlPanelSlotIdx = 0;
        public static final int PIDLoopIdx = 0;
        public static final int timeoutMs = 30;
        public static boolean sensorPhase = true;
        public static boolean motorInvert = false;
        public static final Gains gains = new Gains(0.15, 0.0, 0.0, 0.0, 0, 1.0);
    }
    public static final class intakeConstants {
        public static final int intakeMotor = 9;
        public static final int intakeSolenoid = 6;
        public static final int intakeSolenoid2 = 7;
    }
    public static final class pwmConstants {
        public static final int blinkin_0 = 0;
        public static final int blinkin_1 = 1;
    }
    public static final class digitalIOConstants {
        // assign digital IO (DIO) ports 0-9
        public static final int dio0_indexerSensor1 = 0;
        public static final int dio1_indexerSensor2 = 1;
        public static final int dio2_indexerSensor3 = 2;
        public static final int dio7_turretLimit = 7;
    }
}
