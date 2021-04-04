package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  String[] speedRows = new String[] { "leftDrivePrimary", "rightDrivePrimary" };

  String[] positionRows = new String[] { "leftDrivePrimary", "rightDrivePrimary" };

  String[] drivetrainRows = new String[] { "leftVoltage", "rightVoltage" };

  String[] gyro_logRows = new String[] { "headingDegrees" };

  // FRCLogger speeds = new FRCLogger("speeds.csv", this.speedRows);

  // FRCLogger positions = new FRCLogger("positions.csv", this.positionRows);

  // FRCLogger drivetrain = new FRCLogger("drivetrain.csv", this.drivetrainRows);

  // FRCLogger gyro_log = new FRCLogger("gryo.csv", this.gyro_logRows);

  TalonFX leftDrivePrimary = new TalonFX(4);

  TalonFX leftDriveBack = new TalonFX(5);

  TalonFX rightDrivePrimary = new TalonFX(6);

  TalonFX rightDriveBack = new TalonFX(7);

  AHRS navx = new AHRS(SPI.Port.kMXP);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(25.0D));

  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.596D, 1.49D, 0.0951D);

  PIDController leftPIDController = new PIDController(Constants.kP, 0.0D, Constants.kD);

  PIDController rightPIDController = new PIDController(Constants.kP, 0.0D, Constants.kD);

  Pose2d pose;

  public DriveSubsystem() {
    this.leftDrivePrimary.setInverted(true);
    this.leftDriveBack.setInverted(true);
    this.rightDrivePrimary.setInverted(false);
    this.rightDriveBack.setInverted(false);
    this.leftDrivePrimary.setNeutralMode(NeutralMode.Coast);
    this.leftDriveBack.setNeutralMode(NeutralMode.Coast);
    this.rightDrivePrimary.setNeutralMode(NeutralMode.Coast);
    this.rightDriveBack.setNeutralMode(NeutralMode.Coast);
    this.leftDriveBack.follow((IMotorController) this.leftDrivePrimary);
    this.rightDriveBack.follow((IMotorController) this.rightDrivePrimary);
    resetEncoders();
  }

  public double getYaw() {
    double yaw = this.navx.getYaw();
    return Math.IEEEremainder(yaw, 360.0D) * -1.0D;
  }

  public Rotation2d getHeading() {
    double heading = this.navx.getYaw();
    return Rotation2d.fromDegrees(Math.IEEEremainder(heading, 360.0D) * -1.0D);
  }

  public PIDController getLeftPIDController() {
    return this.leftPIDController;
  }

  public PIDController getRightPIDController() {
    return this.rightPIDController;
  }

  public void SetCoast() {
    this.leftDrivePrimary.setNeutralMode(NeutralMode.Coast);
    this.leftDriveBack.setNeutralMode(NeutralMode.Coast);
    this.rightDrivePrimary.setNeutralMode(NeutralMode.Coast);
    this.rightDriveBack.setNeutralMode(NeutralMode.Coast);
  }

  public void SetBrake() {
    this.leftDrivePrimary.setNeutralMode(NeutralMode.Brake);
    this.leftDriveBack.setNeutralMode(NeutralMode.Brake);
    this.rightDriveBack.setNeutralMode(NeutralMode.Brake);
    this.rightDrivePrimary.setNeutralMode(NeutralMode.Brake);
  }

  public SimpleMotorFeedforward getFeedForward() {
    return this.feedForward;
  }

  public DifferentialDriveKinematics getKinematics() {
    return this.kinematics;
  }

  public void set(double leftVoltage, double rightVoltage) {
    // this.drivetrain.csv.LogWithTime("" + leftVoltage + "," + leftVoltage);
    this.leftDrivePrimary.set(ControlMode.PercentOutput, leftVoltage / 12.0D);
    this.rightDrivePrimary.set(ControlMode.PercentOutput, rightVoltage / 12.0D);
  }

  public void setTeleop(double leftVoltage, double rightVoltage) {
    this.leftDrivePrimary.set(ControlMode.PercentOutput, leftVoltage * -1.0D);
    this.rightDrivePrimary.set(ControlMode.PercentOutput, rightVoltage * -1.0D);
  }

  public void setTeleopLeft(double leftVoltage) {
    System.out.println("left: " + leftVoltage);
    this.leftDrivePrimary.set(ControlMode.PercentOutput, leftVoltage);
  }

  public void setTeleopRight(double rightVoltage) {
    System.out.println("right: " + rightVoltage);
    this.rightDrivePrimary.set(ControlMode.PercentOutput, rightVoltage);
  }

  public Pose2d getPose() {
    return this.pose;
  }

  public void resetEncoders() {
    this.leftDrivePrimary.setSelectedSensorPosition(0.0D);
    this.rightDrivePrimary.setSelectedSensorPosition(0.0D);
  }

  public void resetHeading() {
    this.navx.zeroYaw();
  }

  public void resetOdometryTo(Pose2d pose) {
    this.odometry.resetPosition(pose, getHeading());
  }

  public void resetOdometry() {
    this.odometry.resetPosition(new Pose2d(), getHeading());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        this.leftDrivePrimary.getSelectedSensorVelocity() * 0.07288629737609328D * 10.0D / 2048.0D
            * Units.inchesToMeters(17.27875959474386D),
        this.rightDrivePrimary.getSelectedSensorVelocity() * 0.07288629737609328D * 10.0D / 2048.0D
            * Units.inchesToMeters(17.27875959474386D));
  }

  public float GetCompass() {
    return this.navx.getCompassHeading();
  }

  public void periodic() {
    this.pose = this.odometry.update(getHeading(),
        this.leftDrivePrimary.getSelectedSensorPosition() / 2048.0D * 0.07288629737609328D
            * Units.inchesToMeters(17.27875959474386D),
        this.rightDrivePrimary.getSelectedSensorPosition() / 2048.0D * 0.07288629737609328D
            * Units.inchesToMeters(17.27875959474386D));
    SmartDashboard.putNumber("left encoder pos", this.leftDrivePrimary.getSelectedSensorPosition());
    SmartDashboard.putNumber("right encoder pos", this.rightDrivePrimary.getSelectedSensorPosition());
    SmartDashboard.putNumber("heading", getYaw());
    SmartDashboard.putNumber("left encoder vel", this.leftDrivePrimary.getSelectedSensorVelocity());
    SmartDashboard.putNumber("right encoder vel", this.rightDrivePrimary.getSelectedSensorVelocity());
    // this.speeds.csv.LogWithTime("" +
    // this.leftDrivePrimary.getSelectedSensorVelocity() + ","
    // + this.leftDrivePrimary.getSelectedSensorVelocity());
    // this.gyro_log.csv.LogWithTime(Double.valueOf(getHeading().getDegrees()));
    // this.positions.csv.LogWithTime("" +
    // this.leftDrivePrimary.getSelectedSensorPosition() + ","
    // + this.leftDrivePrimary.getSelectedSensorPosition());

    SmartDashboard.putNumber("X pose", this.odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Y pose", this.odometry.getPoseMeters().getTranslation().getY());
    SmartDashboard.putString("Compass Heading", "" + GetCompass());

    this.leftPIDController.setP(SmartDashboard.getNumber("kP", Constants.kP));
    this.rightPIDController.setP(SmartDashboard.getNumber("kP", Constants.kP));
    this.leftPIDController.setD(SmartDashboard.getNumber("kD", Constants.kD));
    this.rightPIDController.setD(SmartDashboard.getNumber("kD", Constants.kD));
    this.leftPIDController.setI(SmartDashboard.getNumber("kI", Constants.kI));
    this.rightPIDController.setI(SmartDashboard.getNumber("kI", Constants.kI));

    // this.dash.SendRobotPosition(this.odometry.getPoseMeters(),
    // getHeading().getDegrees());
  }
}