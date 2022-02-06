// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.GroupMotorControllers;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX leftLeader = new WPI_TalonFX(3);
  private WPI_TalonFX rightLeader = new WPI_TalonFX(1);
  private WPI_TalonFX leftFollower = new WPI_TalonFX(4);
  private WPI_TalonFX rightFollower = new WPI_TalonFX(2);
  private MotorControllerGroup leftGroup = new MotorControllerGroup(leftLeader,leftFollower);
  private MotorControllerGroup rightGroup = new MotorControllerGroup(rightLeader,rightFollower);
  /*private CANSparkMax leftLeader = new CANSparkMax(Constants.MotorLeft1ID, MotorType.kBrushless);
  private CANSparkMax rightLeader = new CANSparkMax(Constants.MotorRight1ID,MotorType.kBrushless);
  private CANSparkMax leftFollower = new CANSparkMax(Constants.MotorLeft2ID,MotorType.kBrushless);
  private CANSparkMax rightFollower = new CANSparkMax(Constants.MotorRight2ID,MotorType.kBrushless);*/
  //double encoderConstant = (1/10.71)*.155* Math.PI;
  private final DifferentialDrive Ddrive = new DifferentialDrive(leftGroup,rightGroup); 
  /*private final RelativeEncoder leftLEncoder = leftLeader.getEncoder();
  private final RelativeEncoder rightLEncoder = rightLeader.getEncoder();
  private final RelativeEncoder leftFEncoder = leftFollower.getEncoder();
  private final RelativeEncoder rightFEncoder = rightFollower.getEncoder();*/
  private final AHRS NavX = new AHRS(SPI.Port.kMXP);
  private final DifferentialDriveOdometry ddOdometry;
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

    ddOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }
  public double getHeading(){
    return Math.IEEEremainder(NavX.getAngle(),360)* (Constants.kgyroReversed ? -1.0:1.0);
  }
  public double falconUnitsToMeters(double SensorCounts){
    double motorRoatations = (double)SensorCounts/Constants.CountsPerRev;
    double wheelRotations = motorRoatations/Constants.GearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(Constants.WheelRadiusIn));
    return positionMeters;
  }
  public double getLeftDistance(){
    double motorRoations = leftLeader.getSelectedSensorPosition()/Constants.CountsPerRev;
    double wheelRotations = motorRoations/Constants.GearRatio;
    double positionMeters = wheelRotations * (2*Math.PI*Units.inchesToMeters(Constants.WheelRadiusIn));
    return positionMeters;
  }
  public double getRightDistance(){
    double motorRoations = (rightLeader.getSelectedSensorPosition())/Constants.CountsPerRev;
    double wheelRotations = motorRoations/Constants.GearRatio;
    double positionMeters = wheelRotations*(2*Math.PI*.0508);
    return positionMeters;  
  }
  public double getLeftVelocity(){
    return 10*getLeftDistance();  
  }
  public double getRightVelocity(){
    return 10*getRightDistance();  
  }
  @Override
  public void periodic() {
    ddOdometry.update(Rotation2d.fromDegrees(getHeading()), getLeftDistance(), getRightDistance());
    SmartDashboard.putNumber("Left Distance", getLeftDistance());
    SmartDashboard.putNumber("right Distance", getRightDistance());
    SmartDashboard.putNumber("Left velocity", getLeftVelocity());
    SmartDashboard.putNumber("right velocity", getRightVelocity());
  }
  public Pose2d getPose(){
    return ddOdometry.getPoseMeters();
  }
  public DifferentialDriveWheelSpeeds getWheelSpeds(){
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(),getRightVelocity());
  }
  public void drive(XboxController controller, double speed){
    Ddrive.arcadeDrive(controller.getRawAxis(Constants.XboxLeftYaxis), controller.getRawAxis(Constants.XboxRightXaxis));
  }
  public void setMaxOutput(double maxOutput){
    Ddrive.setMaxOutput(maxOutput);
  }
  public void voltDrive(double leftVolts,double rightVolts){
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
    Ddrive.feed();
  }
  public double getTurnRate(){
    return NavX.getRate() * (false ? -1.0 :1.0);
  }
  public void resetEncoders(){
    leftLeader.setSelectedSensorPosition(0);
    leftFollower.setSelectedSensorPosition(0);
    rightFollower.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
  }
}
