// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Ports on the driver station where the contollers are connected
    public static final int DRIVER_CONTROLLER = 0;
    public static final int MANIPULATOR_CONTROLLER = 1;

    // Drivetrain subsystem controllers
    public static final int RIGHT_FRONT_DRIVE_MOTOR = 1;
    public static final int RIGHT_REAR_DRIVE_MOTOR = 2;
    public static final int LEFT_FRONT_DRIVE_MOTOR = 3;
    public static final int LEFT_REAR_DRIVE_MOTOR = 4;

    // Drivetrain PID parameters
    public static final int DRIVETRAIN_MOTOR_PID_LOOP = 0;
    public static final int DRIVETRAIN_MOTOR_PID_TIMEOUT = 0;

    // Drivetrain rate limiters
    // Slew rate limiters
    public static final double SLEW_RATE_LIMIT_ROTATE = 0.5;
    public static final double SLEW_RATE_LIMIT_ACCEL = 0.5;

}
