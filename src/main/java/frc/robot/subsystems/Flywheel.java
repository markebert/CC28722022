// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
  private final CANSparkMax Flywheelmotor = new CANSparkMax(Constants.FlywheelMotor, MotorType.kBrushless);
  private final LinearSystem<N1,N1,N1> m_flywheelPlant = 
  LinearSystemId.identifyVelocitySystem(Constants.kFlywheelKV, Constants.kFlywheelKA);
  private final KalmanFilter<N1, N1, N1> m_observer = 
    new KalmanFilter<>(
      Nat.N1(), 
      Nat.N1(),
      m_flywheelPlant,
      VecBuilder.fill(3.0),//can increase this to causes filted to distrust our state estimate and favor new measurments more highly
      VecBuilder.fill(0.01),// increase this to do the oppsite of the above 
      .020);
  private final LinearQuadraticRegulator<N1,N1,N1>m_controller = 
    new LinearQuadraticRegulator<>(
      m_flywheelPlant,
      VecBuilder.fill(8.00), // velocity error tolerance 
      VecBuilder.fill(12.0),// Control effort (voltage)  tolerance
      .020);
  private final LinearSystemLoop<N1,N1,N1> m_Loop = 
    new LinearSystemLoop<>(m_flywheelPlant,
    m_controller,
    m_observer,
    12.0,
    .020);
  private final RelativeEncoder flywheelEncoder = Flywheelmotor.getEncoder();
  /** Creates a new Flywheel. */
  public Flywheel() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
