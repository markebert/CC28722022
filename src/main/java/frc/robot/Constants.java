// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int MotorLeft1ID = 3;
    public static final int MotorLeft2ID = 4;
    public static final int MotorRight1ID = 1;
    public static final int MotorRight2ID = 2;
    public static final double ksVolts = 0.54773;
    public static final double kvVoltSecondsPerMeter = 0.043784;
    public static final double kaVoltSecondsSqaredPerMeter = 0.0022725;
    public static final double kPDriveVel = 0.093138;
    public static final double kTrackwidthMeter = 0.7112;
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final boolean kgyroReversed = true;
    public static final int XboxRightXaxis = 4;
    public static final int XboxLeftYaxis = 1;
    public static final double dtSpeed = -1;
    public static final int DriveJoystick = 0;
    public static final double CountsPerRev = 2048;
    public static final double GearRatio = 7.75;
    public static final double WheelRadiusIn = 2;
    public static final double kFlywheelKV = 0;
    public static final double kFlywheelKA = 0;
    public static final int FlywheelMotor = 0;
    public static final double IntakeSpeed = -1;
    public static final double OuttakeSpeed = 1;
    public static final double msPerSecond = 1000;
    public static final double k100msPerSecond = 10;

}
