// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.LiftConstants;

public class LiftSubsystem extends SubsystemBase {
    TalonFX m_lift_motor1 = new TalonFX(LiftConstants.lift_motor1_id);
    TalonFX m_lift_motor2 = new TalonFX(LiftConstants.lift_motor2_id);

  private final ElevatorFeedforward elevFF = new ElevatorFeedforward(
    0,0,0,0
  );

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineLift = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setVoltage(output),
            null,
            this
        )
    );

  /** Creates a new LiftSubsystem. */
  public LiftSubsystem() {
    TalonFXConfiguration lift_motor_config = new TalonFXConfiguration();
    //Set lift motor pid values
    lift_motor_config.Slot0.kP = 1;
    lift_motor_config.Slot0.kI = 0;
    lift_motor_config.Slot0.kD = 0;

    //MotorOutputConfigs lift_motor_output_config = new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);
    //lift_motor_config1.withMotorOutput(lift_motor_output_config);
    
    m_lift_motor1.getConfigurator().apply(lift_motor_config);
    m_lift_motor2.getConfigurator().apply(lift_motor_config);
    m_lift_motor2.setControl(new Follower(LiftConstants.lift_motor1_id, true));

  }

  public void setPower(double liftPower)
  {
    m_lift_motor1.set(liftPower);
    //m_lift_motor2.set(liftPower);
  }

  public void setVoltage(Voltage inputVoltage)
  {
    m_lift_motor1.setVoltage(inputVoltage.magnitude());
    //m_lift_motor2.setVoltage(inputVoltage.magnitude());
  }

  public void turnOff()
  {
    m_lift_motor1.set(0);
    m_lift_motor2.set(0);
  }

  public double getPos()
  {
    return m_lift_motor1.getPosition(true).getValueAsDouble();
  }

  public void zeroLiftPos()
  {
    m_lift_motor1.setPosition(0);
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
    SmartDashboard.putNumber("Lift Position", getPos());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
