// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Teleop extends CommandBase {
  private DriveSubsystem drive;
  private DoubleSupplier m_left;
  private DoubleSupplier m_right;

  /** Creates a new Teleop. */
  public Teleop(DriveSubsystem drive, DoubleSupplier left, DoubleSupplier right) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.m_left = left;
    this.m_right = right;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.SetBrake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setTeleop(m_left.getAsDouble(), m_right.getAsDouble());
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
