// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class ChuteSubsystem extends SubsystemBase {
    TalonFX m_chute_motor = new TalonFX(ManipulatorConstants.chute_motor_id);

  /** Creates a new IntakeSubsystem. */
  public ChuteSubsystem() {    
    m_chute_motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake));

  }

  public void setPower(double pow)
  {
    m_chute_motor.set(pow);
  }

  public void turnOff()
  {
    m_chute_motor.set(0);
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
    SmartDashboard.putNumber("test", (SmartDashboard.getNumber("Lift Position", -999)));

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
