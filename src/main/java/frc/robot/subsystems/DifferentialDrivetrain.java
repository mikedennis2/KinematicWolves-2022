// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DifferentialDrivetrain extends SubsystemBase {
  public SlewRateLimiter rotationFilter = new SlewRateLimiter(Constants.SLEW_RATE_LIMIT_ROTATE);
  public SlewRateLimiter accelerationFilter = new SlewRateLimiter(Constants.SLEW_RATE_LIMIT_ACCEL);

  private final WPI_TalonFX m_leftFront = new WPI_TalonFX(Constants.LEFT_FRONT_DRIVE_MOTOR);
  private final WPI_TalonFX m_leftRear = new WPI_TalonFX(Constants.LEFT_REAR_DRIVE_MOTOR);
  private final WPI_TalonFX m_rightFront = new WPI_TalonFX(Constants.RIGHT_FRONT_DRIVE_MOTOR);
  private final WPI_TalonFX m_rightRear = new WPI_TalonFX(Constants.RIGHT_REAR_DRIVE_MOTOR);

  private final MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_leftFront, m_leftRear);
  private final MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_rightFront, m_rightRear);

  private final DifferentialDrive drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = 0.381 * 2; // meters
  private static final double kWheelRadius = 0.0508; // meters
  private static final int kEncoderResolution = 4096;

  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(27.0)); // Distance
                                                                                                                // between
                                                                                                                // two
                                                                                                                // sets
                                                                                                                // of
                                                                                                                // wheels
                                                                                                                // (inches)

  /** Creates a new DifferentialDrivetrain. */
  public DifferentialDrivetrain() {
  }

  private void configureDriveMotorFeedback() {
    m_leftFront.configFactoryDefault();

    m_leftFront.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, Constants.DRIVETRAIN_MOTOR_PID_LOOP,
        Constants.DRIVETRAIN_MOTOR_PID_TIMEOUT);
    m_leftFront.config_kF(Constants.DRIVETRAIN_MOTOR_PID_LOOP, 0);
    m_leftFront.config_kP(Constants.DRIVETRAIN_MOTOR_PID_LOOP, 0);
    m_leftFront.config_kI(Constants.DRIVETRAIN_MOTOR_PID_LOOP, 0);
    m_leftFront.config_kD(Constants.DRIVETRAIN_MOTOR_PID_LOOP, 0);

    m_rightFront.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero,
        Constants.DRIVETRAIN_MOTOR_PID_TIMEOUT);

    m_rightFront.configFactoryDefault();
    m_rightFront.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, Constants.DRIVETRAIN_MOTOR_PID_LOOP,
        Constants.DRIVETRAIN_MOTOR_PID_TIMEOUT);
    m_rightFront.config_kF(Constants.DRIVETRAIN_MOTOR_PID_LOOP, 0);
    m_rightFront.config_kP(Constants.DRIVETRAIN_MOTOR_PID_LOOP, 0);
    m_rightFront.config_kI(Constants.DRIVETRAIN_MOTOR_PID_LOOP, 0);
    m_rightFront.config_kD(Constants.DRIVETRAIN_MOTOR_PID_LOOP, 0);

    m_rightFront.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero,
        Constants.DRIVETRAIN_MOTOR_PID_TIMEOUT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveWithJoysticks(XboxController driverController) {

    // Get axis values for speed and rotational speed
    double xSpeed = driverController.getLeftY();
    double zRotation_rate = -1 * driverController.getLeftX();

    accelerationFilter.calculate(xSpeed);
    rotationFilter.calculate(zRotation_rate);

    // Drive Robot with commanded linear velocity and yaw rate commands
    drive.arcadeDrive(xSpeed, zRotation_rate);

    SmartDashboard.putNumber("X speed commanded by driver", driverController.getLeftY());
    SmartDashboard.putNumber("zRotation Rate Commanded by driver", driverController.getLeftX());
  }
}
