// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Conveyor;

public class Vomit extends CommandBase {
  private BooleanSupplier m_vomit;
  private Conveyor m_conveyor;
  private Collector m_collector;

  /** Creates a new Vomit. */
  public Vomit(Conveyor conveyor, Collector collector) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_collector = collector;
    this.m_conveyor = conveyor;

    addRequirements(m_collector);
    addRequirements(m_conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.m_collector.setCollectorSpeed(-1);
    this.m_conveyor.setConveyorSpeed(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_collector.setCollectorSpeed(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
