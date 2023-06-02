
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive_Train;

public class Robot_Drive extends CommandBase {
  private final Drive_Train _drivetrain;
  private final Joystick _joystick;

  /** Creates a new Robot_Drive. */
  public Robot_Drive(Drive_Train drivetrain, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    _drivetrain = drivetrain;
    _joystick = joystick;
    addRequirements(_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _drivetrain.teleopDrive(_joystick);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
