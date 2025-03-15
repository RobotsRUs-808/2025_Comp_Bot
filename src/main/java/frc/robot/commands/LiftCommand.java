// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.LiftSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class LiftCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LiftSubsystem lift_subsystem;
  private double lift_pow;
  /**
   * Creates a new LiftCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LiftCommand(LiftSubsystem subsystem, double input_pow) {
    lift_subsystem = subsystem;
    lift_pow = input_pow;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if (lift_pow > 0){
    //  lift_subsystem.setPower(lift_pow);
    //}
    //else
    //{
    //  if (lift_subsystem.getLimitSwitch())
    //    lift_subsystem.setPower(0);
    //  else
    //    lift_subsystem.setPower(lift_pow);
    //}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lift_subsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    lift_subsystem.setPower(0);
    return false;
  }
}
