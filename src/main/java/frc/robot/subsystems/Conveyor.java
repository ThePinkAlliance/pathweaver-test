package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {
  public CANSparkMax conveyorMotor = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
  
  public CANEncoder conveyorEncoder = new CANEncoder(this.conveyorMotor);
  
  public DigitalInput breakbeam1 = new DigitalInput(9);
  
  public DigitalInput breakbeam2 = new DigitalInput(8);
  
  public DigitalInput breakbeam3 = new DigitalInput(5);
  
  public void periodic() {
    SmartDashboard.putBoolean("Conveyor Max Capacity", getMagazineCapacity());
  }
  
  public boolean getBreakbeam() {
    return (!this.breakbeam1.get() || !this.breakbeam2.get());
  }
  
  public boolean getMagazineCapacity() {
    return !this.breakbeam3.get();
  }
  
  public void setConveyorSpeed(double speed) {
    SmartDashboard.putNumber("Conveyor Velocity", this.conveyorEncoder.getVelocity());
    this.conveyorMotor.set(speed);
  }
}
