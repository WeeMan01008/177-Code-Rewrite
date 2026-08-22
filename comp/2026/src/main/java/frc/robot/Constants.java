// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
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

    public static class hoodConstants {
        // Constant for hood motor ID
        public static final int HOOD_MOTOR_ID = 21;

        public static final double[] HOOD_POSITION_SHOOTING = { -0.25, -0.5, -0.75, -1, -1.5, -2, -2.2, -2.4, -2.6,
                -3.3, -3.6, -3.8, -4.2, -4.5, -5.0, -5.3, -5.5 };
        public static final double[] HOOD_POSITION_PASSING = { -3.2, -3.4, -3.6, -3.8, -4, -4.2, -4.4, -4.6, -4.8,
                -5 };

        public static final double HOOD_TEST_POSITION = -5.5;

        public static final double HOOD_DOWN = -0.272;

        public static final double HOOD_UP = -6.488;

        public static final double HOOD_STATOR_LIMIT = 60;
        public static final double HOOD_SUPPLY_LIMIT = 40;
        public static final double forwardSoftwareLimit = -6.2;
        public static final double reverseSoftwareLimit = 0;
    }

    public static class shooterConstants {
        // Constants for drum shooter motor ID's, listed by view from behind robot
        public static final int TOPLEFT_SHOOTER_MOTOR_ID = 13;
        public static final int BOTTOMLEFT_SHOOTER_MOTOR_ID = 20;

        public static final int TOPRIGHT_SHOOTER_MOTOR_ID = 11;
        public static final int BOTTOMRIGHT_SHOOTER_MOTOR_ID = 14;

        public static final double SHOOTER_DEBOUNCE_TIME = 0.1;
        public static final double SPINUP_VELOCITY_TOLERANCE = 4;

        public static final int STATOR_CURRENT_LIMIT = 80;
        public static final int SUPPLY_CURRENT_LIMIT = 60;

        public static final double[] SHOOTING_DISTANCES = { 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130,
                140, 150, 160 };

        public static final double[] SHOOTER_SPEEDS = { 33.5, 33.5, 33.5, 33.5, 34.5, 36.25, 36.25, 37, 37.5, 38,
                38.5, 39, 39.25, 40, 40.5, 41, 41 };

        public static final double[] PASSING_DISTANCES = { 130, 140, 150, 160, 170, 180, 190, 200, 210, 220 };

        public static final double[] PASSING_SPEEDS = { 31, 32, 33, 34, 35, 36, 38, 40, 41, 42 };

        public static final double SHOOTING_TEST_SPEEDS = 35;
        public static final double UNJAM_VELOCITY = 30;

        public static final double SHOOTING_SPINUP = 1;
        public static final double PASSING_SPINUP = 0.75;

        public static final double SHOOTING_OFFSET = 23.5 + 21; // 23.5 is half of hub and ___ is half of robot

    }

    public static class intakeConstants {
        // Constants for intake motor ID's
        public static final int INTAKE_DEPLOY_MOTOR_ID = 10;
        public static final int INTAKE_ROLLERLEFT_MOTOR_ID = 18;
        public static final int INTAKE_ROLLERRIGHT_MOTOR_ID = 19;

        public static final int INTAKE_STATOR_LIMIT = 40;
        public static final int INTAKE_SUPPLY_LIMIT = 40;

        public static final int INTAKE_DEPLOY_STATOR_LIMIT = 80;
        public static final int INTAKE_DEPLOY_SUPPLY_LIMIT = 80;

        public static final double INTAKE_VELOCITY = 70;
        public static final double OUTTAKE_VELOCITY = 70;

        public static final double INTAKE_DEPLOYED_POSITION = 11;
        public static final double INTAKE_STORED_POSITION = 0;

    }

    public static class hotdogConstants {
        // Constant for hotdog motor ID
        public static final int HOTDOG_MOTOR_ID = 16;

        public static final double INTAKE_VELOCITY = 30;
        public static final double OUTTAKE_VELOCITY = 60;

        public static final double HOTDOG_STATOR_LIMIT = 40;
        public static final double HOTDOG_SUPPLY_LIMIT = 40;
    }

    public static class carwashConstants {
        // Constants for carwash motor ID's
        public static final int CARWASH_WASHER_MOTOR_ID = 15;

        public static final double CARWASH_SPIN_VELOCITY = 35;
        public static final double CARWASH_SPIN_REVERSE_VELOCITY = 30;
        public static final double CARWASH_OUTTAKE_VELOCITY = 35;

        public static final double CARWASH_STATOR_LIMIT = 80;
        public static final double CARWASH_SUPPLY_LIMIT = 60;
    }
}
