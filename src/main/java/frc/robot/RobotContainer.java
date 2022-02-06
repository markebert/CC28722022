// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;
import java.util.List;

import com.fasterxml.jackson.databind.cfg.ContextAttributes;
import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.OuttakeBall;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain dt = new DriveTrain();
  private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(dt);
  public static XboxController driverController = new XboxController(Constants.DriveJoystick);
  DifferentialDriveVoltageConstraint autoVoltageConstraint;
  TrajectoryConfig config;
  String trajectoryJSON="paths/.json";
  Path trajectoryPath;
  Trajectory trajectory;
  private final Intake intake;  
  private final IntakeBall intakeBall;
  private final OuttakeBall outtakeBall;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    CommandScheduler.getInstance().setDefaultCommand(dt, driveWithJoysticks);
    // Configure the button bindings
    intake = new Intake();
    intakeBall = new IntakeBall(intake);
    intakeBall.addRequirements(intake);
    outtakeBall = new OuttakeBall(intake);
    outtakeBall.addRequirements(intake);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton intakeButton = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
    intakeButton.whileHeld(new IntakeBall(intake));
    JoystickButton outtakeButton = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
    outtakeButton.whileHeld(new OuttakeBall(intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /*autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter),
       Constants.kDriveKinematics,
        5);
    config =
    new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, 
    Constants.kMaxAccelerationMetersPerSecondSquared)
    .setKinematics(Constants.kDriveKinematics)
    .addConstraint(autoVoltageConstraint);
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0,0,new Rotation2d(0)),
    List.of(
      new Translation2d(1,0),
      new Translation2d(2,0)),
      new Pose2d(3,0,new Rotation2d(0)),
      config);
    RamseteController disabledRamsete = new RamseteController(){
      public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,double angularVelocityRefRaidiansPersSecond){
        return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRaidiansPersSecond);
      }
    };
   RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory,
      dt::getPose,
      disabledRamsete,
      new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter,Constants.kaVoltSecondsSqaredPerMeter),
      Constants.kDriveKinematics,
      dt::getWheelSpeds,
      new PIDController(Constants.kPDriveVel,0,0),
      new PIDController(Constants.kPDriveVel, 0, 0),
      dt::voltDrive,
      dt
    );
    */
    // An ExampleCommand will run in autonomous
    return driveWithJoysticks;
    //ramseteCommand.andThen(()-> dt.voltDrive(0, 0));
  }
}
