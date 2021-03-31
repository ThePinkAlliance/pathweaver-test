package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Conveyor;

public class ConveyorAutomated extends CommandBase {
  private Conveyor m_conveyor;
  private Collector m_collector;

  public ConveyorAutomated(Conveyor conveyor, Collector collector) {
    this.m_conveyor = conveyor;
    this.m_collector = collector;

    addRequirements(m_conveyor);
    addRequirements(m_collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_collector.setCollectorSpeed(1);

    if (m_conveyor.getMagazineCapacity()) {
      m_conveyor.setConveyorSpeed(0);
    } else if (m_conveyor.getBreakbeam())
      m_conveyor.setConveyorSpeed(1);
    else
      m_conveyor.setConveyorSpeed(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyor.setConveyorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
