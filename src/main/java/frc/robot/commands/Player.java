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

  boolean EOF = false;
  boolean onTime = true;
  double csvTime;
  long startTime;

  /**
   * Creates a new Player.
   */
  public Player(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    System.out.println("top constructor");

    file = new File("/home/lvuser/path.csv");
    this.drive = drive;

    try {
      scanner = new Scanner(file);
      scanner.useDelimiter(",|\\n");
    } catch (Exception err) {
      System.out.println(err.getMessage());
    }

    System.out.println("bottom constructor");

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
      if (onTime) {
        csvTime = scanner.nextDouble();
      }

      double t_delta = csvTime - (this.GetCurrentTime() - startTime);

      System.out.println("execute");

      System.out.println("current time: " + (this.GetCurrentTime() - startTime));
      System.out.println("csv time: " + csvTime);

      if (t_delta <= 0) {
        onTime = true;

        drive.setTeleopLeft(scanner.nextDouble());
        drive.setTeleopRight(scanner.nextDouble());
      } else {
        onTime = false;
      }
    } else {
      if (scanner != null) {
        scanner.close();

        this.EOF = true;
        System.out.println("EOF TRUE");
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setTeleop(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.EOF;
  }
}
