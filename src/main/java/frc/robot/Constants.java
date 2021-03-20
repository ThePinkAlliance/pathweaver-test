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
    public static final double wheelDiameterInches = 5.75;// 18;
    public static final double wheelCircumferenceInches = wheelDiameterInches * Math.PI;
    public static final double encoderTicksPerRev = 2048;

    public static final double kS = 0.596;// 0.65;
    public static final double kV = 1.49;// 1.47;
    public static final double kA = 0.0951;// 0.109;
    public static double kP = 2.0;// 2.9;
    public static double kD = 0.0;
    public static double kI = 0.0;
}
