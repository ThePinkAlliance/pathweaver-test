package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveSubsystem;

public class DoNothing extends CommandBase {
  private DriveSubsystem m_DriveSubsystem;
  
  public DoNothing(DriveSubsystem drive) {
    this.m_DriveSubsystem = drive;
    addRequirements(new Subsystem[] { (Subsystem)drive });
  }
  
  public void initialize() {
    this.m_DriveSubsystem.SetBrake();
  }
  
  public void execute() {
    this.m_DriveSubsystem.set(0.0D, 0.0D);
  }
  
  public void end(boolean interrupted) {}
  
  public boolean isFinished() {
    return true;
  }
}
