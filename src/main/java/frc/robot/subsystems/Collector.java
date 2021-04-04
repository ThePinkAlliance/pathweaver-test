package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Collector extends SubsystemBase {
  CANSparkMax collector = new CANSparkMax(Constants.collectorRollerCANID, CANSparkMaxLowLevel.MotorType.kBrushless);

  CANEncoder collectorEncoder = new CANEncoder(this.collector);

  private double collectorPower = 0.0D;

  public void periodic() {
    SmartDashboard.putNumber("Collector Power", this.collector.get());
    SmartDashboard.putNumber("Collector Velocity: ", this.collectorEncoder.getVelocity());
  }

  public void setCollectorSpeed(double collector_power) {
    this.collectorPower = collector_power * Constants.collectorMotorGain;
    this.collector.set(this.collectorPower);
  }
}
