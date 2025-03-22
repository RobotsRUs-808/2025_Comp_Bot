// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorSetpoints;


public class LiftSubsystem extends SubsystemBase {
    TalonFX m_lift_motor1 = new TalonFX(LiftConstants.lift_motor1_id);
    TalonFX m_lift_motor2 = new TalonFX(LiftConstants.lift_motor2_id);
    //DigitalInput lift_bot_limit_switch = new DigitalInput(LiftConstants.bot_limit_switch_id);
    
    public enum Setpoint {
      kBottom,
      kFeederStation,
      kLevel1,
      kLevel2,
      kLevel3,
      kLevel4;
      }
      private Encoder lift_encoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
      private boolean wasResetByButton = false;
      private boolean wasResetByLimit = false;
      private double elevatorCurrentTarget = ElevatorSetpoints.kBottom;
      private ProfiledPIDController liftPID = new ProfiledPIDController(
          4, 
          0.0, 
          0, 
          new TrapezoidProfile.Constraints(2, 1)
          );
    private double current_trim;

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
    TalonFXConfiguration lift_motor_config = new TalonFXConfiguration().withMotorOutput(
      new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake))
        .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Current.ofBaseUnits(20, Amp))
            .withStatorCurrentLimitEnable(true));
    //MotorOutputConfigs lift_motor_output_config = new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);
    //lift_motor_config1.withMotorOutput(lift_motor_output_config);
    
    m_lift_motor1.getConfigurator().apply(lift_motor_config);
    m_lift_motor2.getConfigurator().apply(lift_motor_config);
    m_lift_motor2.setControl(new Follower(LiftConstants.lift_motor1_id, false));

    zeroLiftPos();

    //Set the encoder to revolutions with 2048 pulses per revolution
    lift_encoder.setDistancePerPulse(0.00048828125);
    lift_encoder.reset();

    //Trim the lift based on initial trim value
    current_trim = Constants.LiftConstants.init_lift_trim;
  }



  public void setPower(double liftPower)
  {
    m_lift_motor1.set(liftPower);
    /* if(liftPower > 1)
      m_lift_motor1.set(liftPower);
    else
      if (lift_bot_limit_switch.get())
        m_lift_motor1.set(Math.abs(liftPower));
      else
        m_lift_motor1.set(liftPower); */
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

  public void zeroLiftPos()
  {
    m_lift_motor1.setPosition(0);
  }

  public double getCurrent() {
    return m_lift_motor1.getTorqueCurrent().getValueAsDouble();
  }

  //public boolean getLimitSwitch() {
  //  return lift_bot_limit_switch.get();
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
    //Function that does the lift movement based elevatorCurrentTarget and profiledPIDController
    moveToSetpoint();

    //Smart Dashboard data
    SmartDashboard.putNumber("Lift External Position", getExternalEncPos());
    SmartDashboard.putNumber("PID Request", liftPID.calculate(lift_encoder.getDistance(), elevatorCurrentTarget));
    SmartDashboard.putNumber("elevatorCurrentTarget", elevatorCurrentTarget);
    SmartDashboard.putNumber("current trim", current_trim);
    //SmartDashboard.putNumber("Lift Position", getPos());
    //SmartDashboard.putBoolean("At Pickup", (LiftConstants.pickup_height - LiftConstants.height_tolerance) < getPos() && getPos() < (LiftConstants.pickup_height + LiftConstants.height_tolerance));
    //SmartDashboard.putBoolean("At L1", (LiftConstants.L1_height - LiftConstants.height_tolerance) < getPos() && getPos() < (LiftConstants.L1_height + LiftConstants.height_tolerance));
    //SmartDashboard.putBoolean("At L2", (LiftConstants.L2_height - LiftConstants.height_tolerance) < getPos() && getPos() < (LiftConstants.L2_height + LiftConstants.height_tolerance));
    //SmartDashboard.putBoolean("At L3", (LiftConstants.L3_height - LiftConstants.height_tolerance) < getPos() && getPos() < (LiftConstants.L3_height + LiftConstants.height_tolerance));
    //SmartDashboard.putBoolean("At L4", (LiftConstants.L4_height - LiftConstants.height_tolerance) < getPos() && getPos() < (LiftConstants.L4_height + LiftConstants.height_tolerance));
  }

  private void moveToSetpoint() {
    //liftClosedLoopController.setReference(elevatorCurrentTarget, ControlType.kPosition);
    m_lift_motor1.setVoltage(liftPID.calculate(lift_encoder.getDistance(), elevatorCurrentTarget+current_trim));
  }

  private void setTrim(double trim) {
    elevatorCurrentTarget += trim;
  }

  public double getExternalEncPos() {
    return lift_encoder.getDistance();
  }

  public Command setPositionCmd(Setpoint setpoint) {
    return runOnce(
        () -> {
          switch (setpoint) {
            case kBottom:
              elevatorCurrentTarget = ElevatorSetpoints.kBottom;
              break;
            case kFeederStation:
              elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
              break;
            case kLevel1:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
              break;
            case kLevel2:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
              break;
            case kLevel3:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
              break;
            case kLevel4:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
              break;
          }
        });
  }

  //trim commands. Modifies the elevatorCurrentTarget by a small amount
  public Command incrementTrimCmd() {
    return runOnce(
        () -> {
          setTrim(current_trim + .01);
        });
  }

  public Command decrementTrimCmd() {
    return runOnce(
        () -> {
          setTrim(current_trim - .01);
        });
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
