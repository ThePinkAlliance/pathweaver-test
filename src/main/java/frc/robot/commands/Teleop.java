// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FRCLogger.FRCLogger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.player.Recorder;

public class Teleop extends CommandBase {
  private DriveSubsystem drive;
  private DoubleSupplier m_left;
  private DoubleSupplier m_right;
  private Recorder recorder;

  /** Creates a new Teleop. */
  public Teleop(DriveSubsystem drive, DoubleSupplier left, DoubleSupplier right, FRCLogger logger) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.m_left = left;
    this.m_right = right;
    this.recorder = new Recorder(logger);

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
    double left = m_left.getAsDouble();
    double right = m_right.getAsDouble();

    double cal_right = right * right;
    double cal_left = left * left;
    
    if (right > 0) {
      cal_right = -cal_right;
    }
    
    if (left > 0) {
      cal_left = -cal_left;
    }
    
    drive.setTeleopRight(cal_right);
    drive.setTeleopLeft(cal_left);

    recorder.WriteData(cal_left, cal_right);
    System.out.println(cal_left + " " + cal_right);
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
