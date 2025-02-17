// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.LiftConstants;

public class LiftSubsystem extends SubsystemBase {
    TalonFX m_lift_motor1 = new TalonFX(LiftConstants.lift_motor1_id);
    TalonFX m_lift_motor2 = new TalonFX(LiftConstants.lift_motor2_id);
  /** Creates a new LiftSubsystem. */
  public LiftSubsystem() {
    //TalonFXConfiguration lift_motor_config1 = new TalonFXConfiguration();
    MotorOutputConfigs lift_motor_output_config = new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);
    //lift_motor_config1.withMotorOutput(lift_motor_output_config);
    m_lift_motor1.getConfigurator().apply(lift_motor_output_config);
  }

  public void setPower(double liftPower)
  {
    m_lift_motor1.set(liftPower);
    m_lift_motor2.set(liftPower);
  }

  public void turnOff()
  {
    m_lift_motor1.set(0);
    m_lift_motor2.set(0);
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  /* public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          
        });
  }
 */

  public Command setLiftPower(double inputPower) {
        return run(() -> this.setPower(inputPower));
    }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
 /*  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  } */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
