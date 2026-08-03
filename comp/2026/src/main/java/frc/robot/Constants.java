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
    public static final int HOOD_MOTOR_ID = 21;

    public static final double[] HOOD_POSITION_SHOOTING = {};
    public static final double[] HOOD_POSITION_PASSING = {};

  }

  public static class shooterConstants{
  //Constants for drum shooter motor ID's, listed by view from behind robot
    public static final int TOPLEFT_SHOOTER_MOTOR_ID = 13;
    public static final int BOTTOMLEFT_SHOOTER_MOTOR_ID = 20;

    public static final int TOPRIGHT_SHOOTER_MOTOR_ID = 11;
    public static final int BOTTOMRIGHT_SHOOTER_MOTOR_ID = 14;

    public static final int STATOR_CURRENT_LIMIT = 70;

    public static final double[] SHOOTING_DISTANCES = {};

    public static final double[] SHOOTER_SPEEDS = {};

    public static final double[] PASSING_DISTANCES = {};

    public static final double[] PASSING_SPEEDS = {};

  }

  public static class intakeConstants{
  //Constants for intake motor ID's
    public static final int INTAKE_DEPLOY_MOTOR_ID = 10;
    public static final int INTAKE_ROLLERLEFT_MOTOR_ID = 18;
    public static final int INTAKE_ROLLERRIGHT_MOTOR_ID = 19;

    public static final int STATOR_CURRENT_LIMIT = 70;

    public static final double INTAKE_VELOCITY = 0;
    public static final double OUTTAKE_VELOCITY = 0;

    public static final double INTAKE_DEPLOYED_POSITION = 11.522;
    public static final double INTAKE_STORED_POSITION = -0.155;

  }

  public static class hotdogConstants{
  //Constant for hotdog motor ID
    public static final int HOTDOG_MOTOR_ID = 16;

  }

  public static class carwashConstants{
  //Constants for carwash motor ID's
    public static final int CARWASH_WASHER_MOTOR_ID = 15;

    public static final double CARWASH_SPIN_VELOCITY = 25;
  }
}
