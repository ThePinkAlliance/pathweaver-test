// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FRCLogger.FRCLogger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.player.Recorder;

public class Teleop extends CommandBase {
  private DriveSubsystem drive;
  private DoubleSupplier m_left;
  private DoubleSupplier m_right;
  // private Recorder recorder;
  // private FRCLogger m_logger;
  private File localFile;
  private FileWriter writer;
  private BufferedWriter bw;
  private long startTime;

  /** Creates a new Teleop. */
  public Teleop(DriveSubsystem drive, DoubleSupplier left, DoubleSupplier right, FRCLogger logger) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.m_left = left;
    this.m_right = right;
    // this.m_logger = logger;
    // this.recorder = new Recorder(this.m_logger);
    this.localFile = new File("/home/lvuser/" + "macro.csv");

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.SetBrake();

    try {
      writer = new FileWriter(localFile);
      bw = new BufferedWriter(writer);
    } catch (IOException err) {
      throw new Error(err);
    }

    // this.recorder.Init();
    SetStartTime();
    System.out.println("NEW TELEOP RECORDED SESSION");
  }

  public long GetCurrentTime() {
    return System.currentTimeMillis();
  }

  public void SetStartTime() {
    startTime = GetCurrentTime();
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

    // recorder.WriteData(cal_left, cal_right);
    long theTime = GetCurrentTime() - startTime;
    try {

      bw.write(theTime + "," + cal_left + "," + cal_right + "\n");
    } catch (IOException ioe) {
      ioe.printStackTrace();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // recorder.close();
    try {
      bw.close();
      writer.close();
    } catch (IOException ioe) {
      ioe.printStackTrace();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
