package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.FRCLogger.FRCLogger;
import frc.robot.commands.ConveyorAutomated;
import frc.robot.commands.DoNothing;
import frc.robot.commands.ResetSensors;
import frc.robot.commands.Teleop;
import frc.robot.commands.Vomit;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TrajectoryBuilder;
import java.util.Objects;
import java.util.function.Supplier;

public class RobotContainer {
  // private final PathweaverDash dash = new PathweaverDash();
  
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  private final FRCLogger logger = new FRCLogger("macro.csv", null);
  
  private final TrajectoryBuilder builder = new TrajectoryBuilder();
  
  private final Joystick m_joystick = new Joystick(0);
  
  private final Collector m_collector = new Collector();
  
  private final Conveyor m_conveyor = new Conveyor();
  
  String[] ramsetRows = new String[] { "leftRef", "leftMeasure", "rightRef", "rightMeasure" };
  
  // FRCLogger ramseteLogger = new FRCLogger("ramset.csv", this.ramsetRows);
  
  SendableChooser<String> pickPath = new SendableChooser();
  
  public RobotContainer() {
    configureButtonBindings();
    this.m_driveSubsystem.resetEncoders();
    this.m_driveSubsystem.resetHeading();
    this.m_driveSubsystem.resetOdometry();
    
    this.pickPath.addOption("Straght", "output/straight.wpilib.json");
    this.pickPath.addOption("left", "output/left.wpilib.json");
    this.pickPath.addOption("right", "output/right.wpilib.json");
    this.pickPath.addOption("qmark", "output/qmark.wpilib.json");
    this.pickPath.addOption("A-Blue", "output/A-Blue.wpilib.json");
    this.pickPath.addOption("A-Red", "output/A-Red.wpilib.json");
    this.pickPath.addOption("B-Blue", "output/B-Blue.wpilib.json");
    this.pickPath.addOption("B-Red", "output/B-Red.wpilib.json");

    SmartDashboard.putData((Sendable) this.pickPath);
    SmartDashboard.putNumber("kP", Constants.kP);
    SmartDashboard.putNumber("kD", Constants.kD);
    SmartDashboard.putNumber("kI", Constants.kI);
  }
  
  private void configureButtonBindings() {
    this.m_conveyor.setDefaultCommand((Command)new ConveyorAutomated(this.m_conveyor, this.m_collector));
    this.m_driveSubsystem.setDefaultCommand((Command)new Teleop(this.m_driveSubsystem, () -> this.m_joystick.getRawAxis(1), () -> this.m_joystick.getRawAxis(3), logger));
    (new JoystickButton((GenericHID)this.m_joystick, 1)).whenPressed((Command)new ResetSensors(this.m_driveSubsystem));
    (new JoystickButton((GenericHID)this.m_joystick, 2)).whenHeld((Command)new Vomit(this.m_conveyor, this.m_collector));
  }
  
  public Command getAutonomousCommand() {
    this.m_driveSubsystem.resetEncoders();
    this.m_driveSubsystem.resetHeading();
    this.m_driveSubsystem.resetOdometry();
    this.m_driveSubsystem.SetCoast();
    TrajectoryConfig config = new TrajectoryConfig(3.97350993D, 2.0D);
    config.setKinematics(this.m_driveSubsystem.getKinematics());
    Trajectory trajectory = null;
    Supplier<Float> heading = () -> Float.valueOf(this.m_driveSubsystem.GetCompass());
    if (((Float)heading.get()).floatValue() >= 205.0F && ((Float)heading.get()).floatValue() <= 215.0F) {
      trajectory = this.builder.ReadTrajectorys("output/A-Red.wpilib.json");
      System.out.println("Path A Red");
    } else if (((Float)heading.get()).floatValue() >= 155.0F && ((Float)heading.get()).floatValue() <= 165.0F) {
      trajectory = this.builder.ReadTrajectorys("output/A-Blue.wpilib.json");
      System.out.println("Path A Blue");
    } else if (((Float)heading.get()).floatValue() >= 175.0F && ((Float)heading.get()).floatValue() <= 185.0F) {
      trajectory = this.builder.ReadTrajectorys("output/B-Blue.wpilib.json");
      System.out.println("Path B Blue");
    } else if (((Float)heading.get()).floatValue() >= 225.0F && ((Float)heading.get()).floatValue() <= 235.0F) {
      trajectory = this.builder.ReadTrajectorys("output/B-Red.wpilib.json");
      System.out.println("Path B Red");
    } else {
      System.err.println("Could Not Determine The Trajectory");
      return (Command)new DoNothing(this.m_driveSubsystem);
    } 
    System.out.println(heading.get());
    RamseteController disabledRamsete = new RamseteController() {
        public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters, double angularVelocityRefRadiansPerSecond) {
          return new ChassisSpeeds(linearVelocityRefMeters, 0.0D, angularVelocityRefRadiansPerSecond);
        }
      };
    NetworkTableEntry m_leftReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_reference");
    NetworkTableEntry m_leftMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_measurement");
    NetworkTableEntry m_rightReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_reference");
    NetworkTableEntry m_rightMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_measurement");
    Objects.requireNonNull(this.m_driveSubsystem);
    Objects.requireNonNull(this.m_driveSubsystem);
    RamseteCommand command = new RamseteCommand(trajectory, this.m_driveSubsystem::getPose, new RamseteController(4.0D, 0.3D), this.m_driveSubsystem.getFeedForward(), this.m_driveSubsystem.getKinematics(), this.m_driveSubsystem::getWheelSpeeds, this.m_driveSubsystem.getLeftPIDController(), this.m_driveSubsystem.getRightPIDController(), (leftVolts, rightVolts) -> {
          this.m_driveSubsystem.set(leftVolts.doubleValue(), rightVolts.doubleValue());
          m_leftMeasurement.setNumber(Double.valueOf(this.m_driveSubsystem.getFeedForward().calculate((this.m_driveSubsystem.getWheelSpeeds()).leftMetersPerSecond)));
          m_leftReference.setNumber(leftVolts);
          m_rightMeasurement.setNumber(Double.valueOf(this.m_driveSubsystem.getFeedForward().calculate((this.m_driveSubsystem.getWheelSpeeds()).rightMetersPerSecond)));
          m_rightReference.setNumber(Double.valueOf(-rightVolts.doubleValue()));
        }, new Subsystem[] { (Subsystem)this.m_driveSubsystem });
    this.m_driveSubsystem.resetOdometryTo(trajectory.getInitialPose());
    return (Command)new ParallelCommandGroup(new Command[] { (Command)new ConveyorAutomated(this.m_conveyor, this.m_collector), (Command)command
          .andThen(() -> this.m_driveSubsystem.set(0.0D, 0.0D), new Subsystem[0]) });
  }
}
