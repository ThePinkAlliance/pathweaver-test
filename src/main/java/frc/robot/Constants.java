/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // first 14 -> 62
    // second 27 -> 62

    public static final double distBetweenWheelsInches = 25;// 26.84603809585759;
    public static final double gearRatio = 1 / 13.72;
    public static final double wheelDiameterInches = 5.5;// 18;
    public static final double wheelCircumferenceInches = wheelDiameterInches * Math.PI;
    public static final double encoderTicksPerRev = 2048;

    public static final int collectorRollerCANID = 10;
    public static double collectorMotorGain = 1;

    public static final int conveyorBeltCANID = 12;

    // DIO Ports
    public static final int breakbeam1DIOPort = 9;
    public static final int breakbeam2DIOPort = 8;
    public static final int breakbeam3DIOPort = 5;

    public static final double kS = 0.596;// 0.65;
    public static final double kV = 1.49;// 1.47;
    public static final double kA = 0.0951;// 0.109;
    public static double kP = 2.0;// 2.9;
    public static double kD = 0.0;
    public static double kI = 0.0;

    // Shooter Constants
    public static double shooterClosePos = 0;
    public static double shooterClosekP = 0.0004;
    public static double shooterClosekI = 6e-7;
    public static double shooterClosekD = 0.00015;
    public static double shooterClosekIz = 500;
    public static double shooterClosekFF = 0.00015;

    public static double shooterMidPos = 0.75;
    public static double shooterMidkP = 0.0004;
    public static double shooterMidkI = 6e-7;
    public static double shooterMidkD = 0.00015;
    public static double shooterMidkIz = 500;
    public static double shooterMidkFF = 0.00015;

    // Forwards
    public static double shooterForFarPos = 0.75;
    public static double shooterForFarkP = 0.0005;
    public static double shooterForFarkI = 5e-7;
    public static double shooterForFarkD = 0.0002;
    public static double shooterForFarkIz = 500;
    public static double shooterForFarkFF = 0.00015;

    // Backwards
    public static double shooterRevFarPos = 0.7;
    public static double shooterRevFarkP = 0.0005;
    public static double shooterRevFarkI = 5e-7;
    public static double shooterRevFarkD = 0.0002;
    public static double shooterRevFarkIz = 500;
    public static double shooterRevFarkFF = 0.00014;

    // Mix
    public static double shooterMixFarPos = 0.72;
    public static double shooterMixFarkP = 0.0005;
    public static double shooterMixFarkI = 5e-7;
    public static double shooterMixFarkD = 0.0002;
    public static double shooterMixFarkIz = 500;
    public static double shooterMixFarkFF = 0.000145;

    public static double shooterkMaxOutput = 1;
    public static double shooterkMinOutput = 0;

    public static double shooterTurretMotorGain = 1;
    public static double shooterkP = 0.1;
    public static double shooterkI = 0;
    public static double shooterkD = 0;
    public static double shooterCloseVelocity = 300;
    public static double shooterFarVelocity = 400;
    public static double shooterCloseVoltage = 0.7;
    public static double shooterFarVoltage = 0.7;
    public static double conveyorSpeed = 0.5;
    public static double collectorCollectSpeed = 0.8;
    public static boolean collectorExtended = true;
    public static boolean collectorRetracted = false;
    public static double turretSpeed = 1;
    public static boolean turretLeft = true;
    public static boolean turretRight = false;

    public static int shooterRotateCANID = 60; // Brushed
    public static int shooterFlywheelCANID = 31; // Brushless

    public static int shooterLeftServoPort = 0;
    public static int shooterRightServoPort = 1;
}
