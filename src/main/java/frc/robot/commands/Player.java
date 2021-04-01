/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.File;
import java.util.Scanner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FRCLogger.FRCLogger;
import frc.robot.subsystems.DriveSubsystem;

public class Player extends CommandBase {
  File file;
  Scanner scanner;
  DriveSubsystem drive;

  long startTime;

  /**
   * Creates a new Player.
   */
  public Player(FRCLogger logger, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.

    logger.Close();

    file = new File("/home/lvuser/macro.csv");
    this.drive = drive;

    try {
      scanner = new Scanner(file);
      scanner.useDelimiter(",|\\n");
    } catch (Exception err) {
      System.out.println(err.getMessage());
    }

    addRequirements(drive);
  }

  public long GetCurrentTime() {
    return System.currentTimeMillis();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = GetCurrentTime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (scanner != null && scanner.hasNextDouble()) {
      double t_delta;
      double input = scanner.nextDouble();

      System.out.println(input);

    }
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
