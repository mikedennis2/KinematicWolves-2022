/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDrivetrain;

public class DriveRobotOpenLoop extends CommandBase {
  /**
   * Creates a new DriveRobotOpenLoop.
   */
    // The subsytem command runs on the following:
    private final DifferentialDrivetrain m_drivetrain_subsystem;
    private final XboxController m_driverController;

  public DriveRobotOpenLoop(DifferentialDrivetrain drivetrainSubsystem, XboxController driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain_subsystem = drivetrainSubsystem;
    addRequirements(m_drivetrain_subsystem);
    m_driverController = driverController;
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_drivetrain_subsystem.moveWithJoysticks(m_driverController);
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