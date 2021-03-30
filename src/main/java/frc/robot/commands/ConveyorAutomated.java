package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Conveyor;

public class ConveyorAutomated extends CommandBase {
  private Conveyor m_conveyor;
  
  private Collector m_collector;
  
  public ConveyorAutomated(Conveyor conveyor, Collector collector) {
    this.m_conveyor = conveyor;
    this.m_collector = collector;
    addRequirements(new Subsystem[] { (Subsystem)this.m_conveyor });
    addRequirements(new Subsystem[] { (Subsystem)this.m_collector });
  }
  
  public void initialize() {}
  
  public void execute() {
    this.m_collector.setCollectorSpeed(1.0D);
    if (this.m_conveyor.getMagazineCapacity()) {
      this.m_conveyor.setConveyorSpeed(0.0D);
    } else if (this.m_conveyor.getBreakbeam()) {
      this.m_conveyor.setConveyorSpeed(1.0D);
    } else {
      this.m_conveyor.setConveyorSpeed(0.0D);
    } 
  }
  
  public void end(boolean interrupted) {
    this.m_conveyor.setConveyorSpeed(0.0D);
  }
  
  public boolean isFinished() {
    return false;
  }
}
