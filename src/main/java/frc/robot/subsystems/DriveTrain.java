// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// FRC Imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// CTRE Imports
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

// NavX Imports
import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase {

  private WPI_TalonFX leftLeader = new WPI_TalonFX(3);
  private WPI_TalonFX rightLeader = new WPI_TalonFX(1);
  private WPI_TalonFX leftFollower = new WPI_TalonFX(4);
  private WPI_TalonFX rightFollower = new WPI_TalonFX(2);
  private MotorControllerGroup leftGroup = new MotorControllerGroup(leftLeader, leftFollower);
  private MotorControllerGroup rightGroup = new MotorControllerGroup(rightLeader, rightFollower);

  private final DifferentialDrive Ddrive = new DifferentialDrive(leftGroup, rightGroup);
  private final DifferentialDriveOdometry driveOdometry;

  private final AHRS navXGyro = new AHRS(SPI.Port.kMXP);

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    rightLeader.setInverted(true);
    rightFollower.setInverted(true);
    leftLeader.setNeutralMode(NeutralMode.Brake);
    rightLeader.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);
    leftLeader.setSelectedSensorPosition(0);
    leftFollower.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
    rightFollower.setSelectedSensorPosition(0);

    driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getGyroHeading()));
  }

  public double getGyroHeading() {
    return Constants.kgyroReversed ? -navXGyro.getAngle() : navXGyro.getAngle();
  }

  /**
   * @param motor        The {@link WPI_TalonFX} motor to retrieve the encoder
   *                     values from.
   * @param countsPerRev The amount of encoder ticks per one rotation of the
   *                     motor.
   * @return The number of rotations the motor has accumulated since the encoder
   *         was last reset.
   */
  public double getMotorRotations(final WPI_TalonFX motor, final double countsPerRev) {
    return motor.getSelectedSensorPosition() / countsPerRev;
  }

  /**
   * @param motor        The {@link WPI_TalonFX} motor to retrieve the encoder
   *                     values from.
   * @param countsPerRev The amount of encoder ticks per one rotation of the
   *                     motor.
   * @param gearRatio    The gear ratio used to determine the amount of wheel
   *                     rotations.
   * @return The number of rotations the wheel has accumulated since the encoder
   *         was last reset.
   */
  public double getWheelRotations(final WPI_TalonFX motor, final double countsPerRev, final double gearRatio) {
    return getMotorRotations(motor, countsPerRev) / gearRatio;
  }

  /**
   * @param motor             The {@link WPI_TalonFX} motor to retrieve the
   *                          encoder values from.
   * @param countsPerRev      The amount of encoder ticks per one rotation of the
   *                          motor.
   * @param gearRatio         The gear ratio used to determine the amount of wheel
   *                          rotations.
   * @param wheelRadiusInches The radius (in inches) of the wheel being used.
   * @return The current distance (in meters) the wheel has traveled since the
   *         encoder was last reset.
   *         <p>
   *         Note: Going in reverse will decrease the accumulated
   *         distance traveled, while going forward will increase it.
   */
  public double getWheelDistance(final WPI_TalonFX motor, final double countsPerRev, final double gearRatio,
      final double wheelRadiusInches) {
    return getWheelRotations(motor, countsPerRev, gearRatio) * 2 * Math.PI * Units.inchesToMeters(wheelRadiusInches);
  }

  /**
   * @param motor        The {@link WPI_TalonFX} motor to retrieve the encoder
   *                     velocity values from.
   * @param countsPerRev The amount of encoder ticks per one rotation of the
   *                     motor.
   * @return The current rotations per minute (RPM) of the motor.
   */
  public double getMotorVelocity(final WPI_TalonFX motor, final double countsPerRev) {
    return (motor.getSelectedSensorVelocity() / countsPerRev) * 600;
  }

  /**
   * @param motor        The {@link WPI_TalonFX} motor to retrieve the encoder
   *                     velocity values from.
   * @param countsPerRev The amount of encoder ticks per one rotation of the
   *                     motor.
   * @param gearRatio    The gear ratio used to determine the amount of wheel
   *                     rotations.
   * @return The current rotations per minute (RPM) of the wheel.
   */
  public double getWheelVelocity(final WPI_TalonFX motor, final double countsPerRev, final double gearRatio) {
    return getMotorVelocity(motor, countsPerRev) / gearRatio;
  }

  /**
   * @param motor             The {@link WPI_TalonFX} motor to retrieve the
   *                          encoder values from.
   * @param countsPerRev      The amount of encoder ticks per one rotation of the
   *                          motor.
   * @param gearRatio         The gear ratio used to determine the amount of wheel
   *                          rotations.
   * @param wheelRadiusInches The radius (in inches) of the wheel being used.
   * @return The current meters per second (M/s) of the wheel.
   */
  public double getWheelVelocityMetersPerSecond(final WPI_TalonFX motor, final double countsPerRev,
      final double gearRatio, final double wheelRadiusInches) {
    final double metersPerMinute = getWheelVelocity(motor, countsPerRev, gearRatio) * 2 * Math.PI
        * Units.inchesToMeters(wheelRadiusInches);
    return metersPerMinute / 60;
  }

  @Override
  public void periodic() {
    driveOdometry.update(Rotation2d.fromDegrees(getGyroHeading()),
        getWheelDistance(leftLeader, Constants.CountsPerRev, Constants.GearRatio, Constants.WheelRadiusIn),
        getWheelDistance(rightLeader, Constants.CountsPerRev, Constants.GearRatio, Constants.WheelRadiusIn));

    SmartDashboard.putString("Left Distance: ", String.format("%.3f M",
        getWheelDistance(leftLeader, Constants.CountsPerRev, Constants.GearRatio, Constants.WheelRadiusIn)));
    SmartDashboard.putString("Right Distance: ", String.format("%.3f M",
        getWheelDistance(rightLeader, Constants.CountsPerRev, Constants.GearRatio, Constants.WheelRadiusIn)));

    SmartDashboard.putString("Left Velocity:", String.format("%.3f M/s", getWheelVelocityMetersPerSecond(leftLeader,
        Constants.CountsPerRev, Constants.GearRatio, Constants.WheelRadiusIn)));
    SmartDashboard.putString("Right Velocity:", String.format("%.3f M/s", getWheelVelocityMetersPerSecond(rightLeader,
        Constants.CountsPerRev, Constants.GearRatio, Constants.WheelRadiusIn)));
  }

  public Pose2d getPose() {
    return driveOdometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        getWheelVelocityMetersPerSecond(leftLeader, Constants.CountsPerRev, Constants.GearRatio,
            Constants.WheelRadiusIn),
        getWheelVelocityMetersPerSecond(rightLeader, Constants.CountsPerRev, Constants.GearRatio,
            Constants.WheelRadiusIn));
  }

  public void drive(XboxController controller) {
    Ddrive.arcadeDrive(controller.getRawAxis(Constants.XboxLeftYaxis), controller.getRawAxis(Constants.XboxRightXaxis));
  }

  public void setMaxOutput(double maxOutput) {
    Ddrive.setMaxOutput(maxOutput);
  }

  public void voltDrive(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
    Ddrive.feed();
  }

  public double getTurnRate() {
    return Constants.kgyroReversed ? -navXGyro.getRate() : navXGyro.getRate();
  }

  public void resetEncoders() {
    leftLeader.setSelectedSensorPosition(0);
    leftFollower.setSelectedSensorPosition(0);
    rightFollower.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
  }

}
