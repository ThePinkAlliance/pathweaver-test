// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrajectoryBuilder extends SubsystemBase {
  /** Creates a new TrajectoryBuilder. */
  public TrajectoryBuilder() {
  }

  public Trajectory ReadTrajectorys(String path) {
    Trajectory trajectory = new Trajectory();

    try {
      Path resolvedPath = Filesystem.getDeployDirectory().toPath().resolve(path);
      trajectory = TrajectoryUtil.fromPathweaverJson(resolvedPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
    }

    return trajectory;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
