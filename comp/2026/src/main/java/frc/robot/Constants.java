// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    
    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

  public static class hoodConstants{
  //Constant for hood motor ID
    public static final int HOOD_MOTOR_ID = 0;

  }

  public static class shooterConstants{
  //Constants for drum shooter motor ID's, listed by view from behind robot
    public static final int TOPLEFT_SHOOTER_MOTOR_ID = 0;
    public static final int BOTTOMLEFT_SHOOTER_MOTOR_ID = 0;

    public static final int TOPRIGHT_SHOOTER_MOTOR_ID = 0;
    public static final int BOTTOMRIGHT_SHOOTER_MOTOR_ID = 0;

    public static final int STATOR_CURRENT_LIMIT = 70;

    public static final double[] SHOOTING_DISTANCES = {};

    public static final double[] SHOOTER_SPEEDS = {};

    public static final double[] PASSING_DISTANCES = {};

    public static final double[] PASSING_SPEEDS = {};

  }

  public static class intakeConstants{
  //Constants for intake motor ID's
    public static final int INTAKE_DEPLOY_MOTOR_ID = 0;
    public static final int INTAKE_ROLLERLEFT_MOTOR_ID = 0;
    public static final int INTAKE_ROLLERRIGHT_MOTOR_ID = 0;

    public static final int STATOR_CURRENT_LIMIT = 70;

  }

  public static class hotdogConstants{
  //Constant for hotdog motor ID
    public static final int HOTDOG_MOTOR_ID = 0;

  }

  public static class carwashConstants{
  //Constants for carwash motor ID's
    public static final int CARWASH_WASHER_MOTOR_ID = 0;
  }
  
  public static class driveConstants{
  //Constants for drive train motor and encoder ID's
    public static final int FRONTLEFT_DRIVE_ID = 0;
    public static final int FRONTLEFT_ANGLE_ID = 0;
    public static final int FRONTLEFT_ENCODER_ID = 0;

    public static final int FRONTRIGHT_DRIVE_ID = 0;
    public static final int FRONTRIGHT_ANGLE_ID = 0;
    public static final int FRONTRIGHT_ENCODER_ID = 0;

    public static final int BACKLEFT_DRIVE_ID = 0;
    public static final int BACKLEFT_ANGLE_ID = 0;
    public static final int BACKLEFT_ENCODER_ID = 0;

    public static final int BACKRIGHT_DRIVE_ID = 0;
    public static final int BACKRIGHT_ANGLE_ID = 0;
    public static final int BACKRIGHT_ENCODER_ID = 0;
  }

}
