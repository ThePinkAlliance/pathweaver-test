package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Conveyor;
import java.util.function.BooleanSupplier;

public class Vomit extends CommandBase {
  private BooleanSupplier m_vomit;
  
  private Conveyor m_conveyor;
  
  private Collector m_collector;
  
  public Vomit(Conveyor conveyor, Collector collector) {
    this.m_collector = collector;
    this.m_conveyor = conveyor;
    addRequirements(new Subsystem[] { (Subsystem)this.m_collector });
    addRequirements(new Subsystem[] { (Subsystem)this.m_conveyor });
  }
  
  public void initialize() {}
  
  public void execute() {
    this.m_collector.setCollectorSpeed(-1.0D);
    this.m_conveyor.setConveyorSpeed(-1.0D);
  }
  
  public void end(boolean interrupted) {
    this.m_collector.setCollectorSpeed(1.0D);
  }
  
  public boolean isFinished() {
    return false;
  }
}
