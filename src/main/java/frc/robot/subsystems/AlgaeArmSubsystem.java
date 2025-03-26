// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class AlgaeArmSubsystem extends SubsystemBase {
    TalonFX m_algae_intake_motor = new TalonFX(ManipulatorConstants.algae_intake_motor_id);
    //TalonFX m_algae_arm_motor = new TalonFX(ManipulatorConstants.algae_arm_motor_id);

  /** Creates a new AlgaeArmSubsystem. */
  public AlgaeArmSubsystem() {    
    m_algae_intake_motor.getConfigurator().apply(
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake));
    //m_algae_arm_motor.getConfigurator().apply(
    //    new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake));

  }

  public void setIntakePower(double pow)
  {
    m_algae_intake_motor.set(pow);
  }

  public void turnIntakeOff()
  {
    m_algae_intake_motor.set(0);
  }

  //public void setArmPower(double pow)
  //{
  //  m_algae_arm_motor.set(pow);
  //}
//
  //public void turnArmOff()
  //{
  //  m_algae_arm_motor.set(0);
  //}

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
