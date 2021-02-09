/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.containers;

import static frc.robot.autonomous.ExtendedTrajectoryUtilities.tryGetDeployedTrajectory;
import static frc.robot.autonomous.GenericAutonUtilities.createDefaultControllerBuilder;
import static frc.robot.utility.ExtendedMath.withHardDeadzone;
import static frc.robot.utility.ExtendedMath.withContinuousDeadzone;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.Autonomous_ForceIndexBallsCommand;
import frc.robot.autonomous.pshoot.Autonomous_PreciseShootingCommand;
import frc.robot.autonomous.pshoot.IPreciseShootingOI;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.OdometricSwerveDashboardUtility;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_AdvancedFollowTrajectoryCommand;
import frc.robot.subsystems.swerve.odometric.factory.OdometricSimulatedSwerveFactory;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Add your docs here.
 */
public class PortfolioDemoContainer implements IRobotContainer{
    private XboxController controller = new XboxController(1);
    private OdometricSwerve swerve = new OdometricSimulatedSwerveFactory().makeSwerve();
    private OdometricSwerveDashboardUtility utility = new OdometricSwerveDashboardUtility(swerve);

    private double xSensitivity = 0, ySensitivity = 0, zSensitivity = 0, xDeadzone = 0.0, yDeadzone = 0.0,
            zDeadzone = 0.0;

    public PortfolioDemoContainer(){
        configureSwerve();
        configureSmartDashboardControls();
        configureAutonomous();
    }

    private void configureSmartDashboardControls() {
        SmartDashboard.putData("Control Preferences", new Sendable() {

            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("X Axis Sensitivity", () -> xSensitivity, value -> xSensitivity = value);
                builder.addDoubleProperty("Y Axis Sensitivity", () -> ySensitivity, value -> ySensitivity = value);
                builder.addDoubleProperty("Z Axis Sensitivity", () -> zSensitivity, value -> zSensitivity = value);
                builder.addDoubleProperty("X Axis Deadzone", () -> xDeadzone, value -> xDeadzone = value);
                builder.addDoubleProperty("Y Axis Deadzone", () -> yDeadzone, value -> yDeadzone = value);
                builder.addDoubleProperty("Z Axis Deadzone", () -> zDeadzone, value -> zDeadzone = value);
            }
        });

        SmartDashboard.putData("Swerve Transform", new OdometricSwerveDashboardUtility(swerve));


    }

    private SendableChooser<Command> autonomousChooser;
    
    private void configureAutonomous() {
        autonomousChooser = new SendableChooser<>();

        autonomousChooser.addOption(
            "Shoot and Cross The Line", 
            createShootAndCrossTheLineCommand());

        autonomousChooser.addOption(
            "Away From Center, Move Forward and Shoot",
            createAwayFromCenterMoveForwardAndShootCommand());

        autonomousChooser.addOption(
            "Away From Center, Move Backward and Shoot",
            createAwayFromCenterMoveBackwardAndShootCommand());

        autonomousChooser.addOption(
            "Citrus Compatible Primary", 
            createCitrusCompatibleCommand());
        autonomousChooser.addOption(
            "Citrus Compatible Secondary",
            createCitrusCompatibleSecondary());
        autonomousChooser.addOption(
            "Trench Citrus Compatible Primary", 
            createTrenchCitrusCompatiblePartACommand());

        autonomousChooser.addOption(
            "Trench Citrus Compatible Second", 
            createTrenchCitrusCompatibleBCommand());

        SmartDashboard.putData("Selected Auto", autonomousChooser);
        SmartDashboard.putData("Run Autonomous Command", new InstantCommand(() -> autonomousChooser.getSelected().schedule()));
        SmartDashboard.putData("Reset Pose", new InstantCommand(() -> swerve.resetPose(new Pose2d())));
    }

    private SequentialCommandGroup createTrenchCitrusCompatibleBCommand() {
        return createTrenchCitrusPart1Command()
            .andThen(
                new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                    swerve,
                    createDefaultControllerBuilder()
                    .withTrajectory(tryGetDeployedTrajectory("TrenchCitrusCompatiblePart2B"))
                    .withEndRotation(new Rotation2d(7 * Math.PI / 6))
                    .buildController()))
            .andThen(new WaitCommand(5))
            .andThen(
                new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                    swerve,
                    createDefaultControllerBuilder()
                    .withTrajectory(tryGetDeployedTrajectory("TrenchCitrusCompatiblePart3B"))
                    .withEndRotation(new Rotation2d(Math.PI))
                    .buildController()))
                .andThen(new WaitCommand(4));
    }

    private SequentialCommandGroup createTrenchCitrusCompatiblePartACommand() {
        return createTrenchCitrusPart1Command()
        .andThen(new OdometricSwerve_AdvancedFollowTrajectoryCommand(
            swerve,
            createDefaultControllerBuilder()
            .withEndRotation(new Rotation2d(Math.PI))
            .withTrajectory(tryGetDeployedTrajectory("TrenchCitrusCompatiblePart2A"))
            .withMaxVelocity(4.0)
            .with_kW(6)
            .buildController()))
        // .andThen(this::aimAtInnerPort, limelight, turret, swerve)

        .andThen(() -> swerve.moveFieldCentric(0, 0, 0), swerve)
        .andThen(new WaitCommand(3));
    }

    private CommandGroupBase createTrenchCitrusPart1Command() {
        return new InstantCommand()//() -> swerve.resetPose(new Pose2d(13, -7.5, new Rotation2d(Math.PI))), swerve)
            // .andThen(this::aimAtInnerPort, turret, limelight, swerve)
            .andThen(new WaitCommand(3))
            .andThen(
                
                (new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                    swerve,
                    createDefaultControllerBuilder()
                    .withEndRotation(new Rotation2d(Math.PI))
                    .with_kW(6)
                    .withRotationsEnabled(true)
                    .withTrajectory(tryGetDeployedTrajectory("TrenchCitrusCompatiblePart1"))
                    .withMaxVelocity(2.0)
                    .buildController())));
    }

    private SequentialCommandGroup createCitrusCompatibleSecondary(){
        return 
        new InstantCommand()//() -> swerve.resetPose(new Pose2d(12.565, -4.875, new Rotation2d(Math.PI))), swerve)
        .andThen(new WaitCommand(3))
        .andThen(new OdometricSwerve_AdvancedFollowTrajectoryCommand(
            swerve, 
            createDefaultControllerBuilder()
            .withEndRotation(new Rotation2d(-1.72, -0.749))
            .with_kW(6)
            .withTrajectory(tryGetDeployedTrajectory("CitrusCompatiblePart1"))
            .withMaxVelocity(4.0)
            .buildController()))
        .andThen(
            new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                swerve, 
                createDefaultControllerBuilder()
                .withEndRotation(new Rotation2d(-1.72,-0.749))
                .with_kW(6)
                .withMaxVelocity(1)
                .withTrajectory(tryGetDeployedTrajectory("CitrusCompatiblePart2")).buildController()))
        .andThen(new OdometricSwerve_AdvancedFollowTrajectoryCommand(
            swerve, 
            createDefaultControllerBuilder()
            .withEndRotation(new Rotation2d(Math.PI))
            .withMaxVelocity(4.0)
            .with_kW(6)
            .withTrajectory(tryGetDeployedTrajectory("CitrusCompatiblePart3"))
            .buildController()));
    }
    private SequentialCommandGroup createCitrusCompatibleCommand() {
        return new InstantCommand()
            .andThen(new InstantCommand(() -> swerve.resetPose(new Pose2d(12.565, -4.875, new Rotation2d(Math.PI))), swerve))
            .andThen(new WaitCommand(1.5))
            .andThen(new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                swerve,
                createDefaultControllerBuilder()
                .withEndRotation(new Rotation2d(Math.PI + Math.PI * 1.2 / 7))
                .withTrajectory(tryGetDeployedTrajectory("CitrusCompatiblePart1"))
                .withMaxVelocity(4.0)
                .buildController()))
            .andThen(() -> swerve.moveFieldCentric(0, 0, 0), swerve)
            .andThen(
                    new RunCommand(() -> swerve.moveFieldCentric(0.1, -0.25 * 2, 0), swerve)
                    .withTimeout(2)
                    .andThen(
                        new RunCommand(() -> swerve.moveFieldCentric(0.1 * 2.2, -0.25 * 2.2, 0), swerve)
                        .withTimeout(0.7)
                        .andThen(
                            new RunCommand(() -> swerve.moveFieldCentric(0, 0, 1))
                            .withTimeout(1))
                        .andThen(
                            new RunCommand(() -> swerve.moveFieldCentric(-0.1, 0, 0))
                            .withTimeout(1))
                        .andThen(new WaitCommand(1))
                        .andThen(
                            new RunCommand(() -> swerve.moveFieldCentric(0, 0, -2))
                            .withTimeout(1))))
            .andThen(
                new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                    swerve,
                    createDefaultControllerBuilder()
                    .withEndRotation(new Rotation2d(Math.PI))
                    .withInitialAllowableTranslationError(0.5)
                    .withFinalAllowableTranslationError(0.02)
                    .withTrajectory(tryGetDeployedTrajectory("CitrusCompatiblePart3"))
                    .withMaxVelocity(4.0)
                    .buildController())
                .withTimeout(0))
            .andThen(() -> swerve.moveFieldCentric(0, 0, 0), swerve)
            .andThen(new WaitCommand(4));
    }

    private SequentialCommandGroup createAwayFromCenterMoveBackwardAndShootCommand() {
        return new InstantCommand()//() -> swerve.resetPose(new Pose2d(13, -2.5, new Rotation2d(Math.PI))), swerve)
            .andThen(
                new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                    swerve,
                    createDefaultControllerBuilder()
                    .withTrajectory(tryGetDeployedTrajectory("AwayFromCenterBackward"))
                    .withEndRotation(new Rotation2d(Math.PI)).buildController()))
            .andThen(() -> swerve.moveFieldCentric(0, 0, 0))
            .andThen(new WaitCommand(2));
    }

    private SequentialCommandGroup createAwayFromCenterMoveForwardAndShootCommand() {
        return new InstantCommand()//() -> swerve.resetPose(new Pose2d(13, -2.5, new Rotation2d(Math.PI))), swerve)
            .andThen(
                new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                    swerve,
                    createDefaultControllerBuilder()
                    .withTrajectory(tryGetDeployedTrajectory("AwayFromCenterForward"))
                    .withEndRotation(new Rotation2d(Math.PI))
                    .buildController()))
            .andThen(() -> swerve.moveFieldCentric(0, 0, 0))
            .andThen(
new WaitCommand(2));
    }

    private SequentialCommandGroup createShootAndCrossTheLineCommand() {
        return new InstantCommand()//() -> swerve.resetPose(new Pose2d(13, -5.75, new Rotation2d(Math.PI))), swerve)
            .andThen(
                new WaitCommand(4)
            )
            .andThen(
                new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                    swerve,
                    createDefaultControllerBuilder()
                    .withEndRotation(new Rotation2d(Math.PI))
                    .withTrajectory(tryGetDeployedTrajectory("CrossTheLine"))
                    .buildController()));
    }
    private void configureSwerve() {
        swerve.setDefaultCommand(createHardDeadzoneSwerveCommand());
    }
    private CommandBase createHardDeadzoneSwerveCommand(){
        return new RunCommand(() -> {
            var forwardSpeed = withHardDeadzone(-controller.getY(Hand.kLeft), yDeadzone) * ySensitivity;
            var leftwardSpeed = withHardDeadzone(-controller.getX(Hand.kLeft), xDeadzone) * xSensitivity;
            var counterClockwardSpeed = withHardDeadzone(-controller.getX(Hand.kRight), zDeadzone) * zSensitivity;
            
            swerve.moveFieldCentric(0, 0, 0);
            
        }, swerve);
    }
}
