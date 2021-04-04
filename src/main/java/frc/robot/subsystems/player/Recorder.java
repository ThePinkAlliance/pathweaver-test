/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.player;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRCLogger.FRCLogger;

public class Recorder extends SubsystemBase {
  String[] rows = { "Time", "drivetrain-left", "drivetrain-right" };
  FRCLogger logger;

  long startTime;

  /**
   * Creates a new Recorder.
   */
  public Recorder(FRCLogger logger) {
    this.logger = logger;
  }

  public void Init() {
    startTime = GetCurrentTime();
    logger.DestructiveCreate();
  }

  public long GetCurrentTime() {
    return System.currentTimeMillis();
  }

  public void SetStartTime() {
    startTime = GetCurrentTime();
  }

  public void close() {
    logger.Close();
  }

  public void WriteData(double left, double right) {
    logger.csv.Write("" + (GetCurrentTime() - startTime) + "," + left + "," + right + "\n");
  }
}
