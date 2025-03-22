// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.ChuteSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LiftSubsystem.Setpoint;
import frc.robot.commands.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController gamepad1 = new CommandXboxController(0);
    private final CommandXboxController gamepad2 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser;

    public LiftSubsystem lift_subsystem = new LiftSubsystem();
    public ChuteSubsystem chuteSubsystem = new ChuteSubsystem();
    //public AlgaeArmSubsystem algaeArmSubsystem = new AlgaeArmSubsystem();

    //private final double liftpow = .75;
    private final Command c_lift_up = new InstantCommand( () -> lift_subsystem.setPower(LiftConstants.lift_manual_speed) );
    private final Command c_lift_down = new InstantCommand( () -> lift_subsystem.setPower(-LiftConstants.lift_manual_speed) );
    private final Command c_lift_off = new InstantCommand( () -> lift_subsystem.setPower(0) );
    private final Command c_lift_zero_enc = new InstantCommand( () -> lift_subsystem.zeroLiftPos() );

    //private final Command c_algae_in = new InstantCommand( () -> algaeArmSubsystem.setIntakePower(ManipulatorConstants.algae_intake_speed));
    //private final Command c_algae_out = new InstantCommand( () -> algaeArmSubsystem.setIntakePower(-ManipulatorConstants.algae_intake_speed));
    //private final Command c_algae_off = new InstantCommand( () -> algaeArmSubsystem.setIntakePower(0));
    //
    //private final Command c_algae_arm_up = new InstantCommand( () -> algaeArmSubsystem.setArmPower(ManipulatorConstants.algae_arm_speed));
    //private final Command c_algae_arm_down = new InstantCommand( () -> algaeArmSubsystem.setArmPower(-ManipulatorConstants.algae_arm_speed));
    //private final Command c_algae_arm_off = new InstantCommand( () -> algaeArmSubsystem.setArmPower(0));
//
    //private final Command c_chute_shoot = new InstantCommand(() -> chuteSubsystem.setPower(1));


    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention. * (30 > 45 ? .1 : 1)
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-gamepad1.getLeftY() * MaxSpeed * (SmartDashboard.getNumber("Lift Position", 101) > 100 ? .1 : 1)) // Drive forward with negative Y (forward)
                    .withVelocityY(-gamepad1.getLeftX() * MaxSpeed * (SmartDashboard.getNumber("Lift Position", 101) > 100 ? .1 : 1)) // Drive left with negative X (left)
                    .withRotationalRate(-gamepad1.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        chuteSubsystem.setDefaultCommand(
            new InstantCommand( 
                () -> chuteSubsystem.setPower((gamepad2.getRightTriggerAxis()-gamepad2.getLeftTriggerAxis()) * ManipulatorConstants.chute_speed), chuteSubsystem
            )
        );
//
        //algaeArmSubsystem.setDefaultCommand(
        //    new InstantCommand( 
        //        () -> algaeArmSubsystem.setArmPower(-gamepad2.getLeftY() * ManipulatorConstants.algae_arm_speed), algaeArmSubsystem
        //    )
        //);

        //lift_subsystem.setDefaultCommand(
        //    new InstantCommand( 
        //        () -> lift_subsystem.setPower(-MathUtil.applyDeadband(gamepad2.getRightY(), .1) * LiftConstants.lift_manual_speed), lift_subsystem
        //    )
        //);


        //lift_subsystem.setDefaultCommand(lift_subsystem.setLiftPower(gamepad1.getRightTriggerAxis()-gamepad1.getLeftTriggerAxis()) );

        /* gamepad1.a().whileTrue(drivetrain.applyRequest(() -> brake));
        gamepad1.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-gamepad1.getLeftY(), -gamepad1.getLeftX()))
        )); */

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        gamepad1.back().and(gamepad1.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        gamepad1.back().and(gamepad1.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        gamepad1.start().and(gamepad1.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        gamepad1.start().and(gamepad1.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        gamepad1.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        /* gamepad2.rightBumper().onTrue(new LiftCommand(lift_subsystem, LiftConstants.lift_manual_speed));
        gamepad2.leftBumper().onTrue(new LiftCommand(lift_subsystem, -LiftConstants.lift_manual_speed));
 */
        // gamepad2.rightBumper().onTrue(c_lift_up).onFalse(c_lift_off);
        // gamepad2.leftBumper().onTrue(c_lift_down).onFalse(c_lift_off);
        gamepad2.povLeft().onTrue(c_lift_zero_enc);

        //gamepad2.a().onTrue(c_algae_in).onFalse(c_algae_off);
        //gamepad2.b().onTrue(c_algae_out).onFalse(c_algae_off);

        //gamepad2.povUp().onTrue(c_algae_arm_up).onFalse(c_algae_arm_off);
        //gamepad2.povDown().onTrue(c_algae_arm_down).onFalse(c_algae_arm_off);

        gamepad2.leftBumper().whileTrue(lift_subsystem.setPositionCmd(Setpoint.kBottom));
        gamepad2.rightBumper().whileTrue(lift_subsystem.setPositionCmd(Setpoint.kFeederStation));
        gamepad2.povDown().whileTrue(lift_subsystem.setPositionCmd(Setpoint.kLevel1));
        gamepad2.povLeft().whileTrue(lift_subsystem.setPositionCmd(Setpoint.kLevel2));
        gamepad2.povRight().whileTrue(lift_subsystem.setPositionCmd(Setpoint.kLevel3));
        gamepad2.povUp().whileTrue(lift_subsystem.setPositionCmd(Setpoint.kLevel4));


        drivetrain.registerTelemetry(logger::telemeterize);
        //gamepad1.rightBumper().whileTrue(lift_subsystem.setLiftPower(1), lift_subsystem.setLiftPower(0));
    }

    public Command getAutonomousCommand() {
        //return Commands.print("No autonomous command configured");
        return autoChooser.getSelected();
    }
}
