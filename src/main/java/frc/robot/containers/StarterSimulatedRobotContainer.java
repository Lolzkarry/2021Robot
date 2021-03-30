// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.containers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.autonomous.ExtendedTrajectoryUtilities;
import frc.robot.autonomous.GenericAutonUtilities;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_AdvancedFollowTrajectoryCommand;
import frc.robot.subsystems.swerve.odometric.factory.OdometricSimulatedSwerveFactory;
import frc.robot.utility.ExtendedMath;

import static frc.robot.autonomous.ExtendedTrajectoryUtilities.tryGetDeployedTrajectory;

/** Add your docs here. */
public class StarterSimulatedRobotContainer implements RobotContainer{
    
    OdometricSwerve swerve = new OdometricSimulatedSwerveFactory().makeSwerve();
    XboxController controller;
    double maxSpeedMeterSec = 3, maxRotationRadSec = 3;
    boolean useXboxController = false;
    double controllerDeadzone = 0.2;
    public StarterSimulatedRobotContainer(){

        if(useXboxController){
            configureXboxController();
        }else{
            swerve.setDefaultCommand(new RunCommand(() -> swerve.moveFieldCentric(0, 0, 0), swerve));
        }

        configureSmartDashboardControls();

        configureAutonomous();
    }
    private void configureXboxController(){
        controller = new XboxController(0);
        swerve.setDefaultCommand(new RunCommand(() -> {
            swerve.moveFieldCentric(
                ExtendedMath.withContinuousDeadzone(controller.getX(Hand.kLeft), controllerDeadzone) * maxSpeedMeterSec, 
                -ExtendedMath.withContinuousDeadzone(controller.getY(Hand.kLeft), controllerDeadzone) * maxSpeedMeterSec, 
                ExtendedMath.withContinuousDeadzone(controller.getX(Hand.kRight), controllerDeadzone) * maxRotationRadSec);
        }, swerve));
    }
    
    private void configureAutonomous(){
        var trajectory = ExtendedTrajectoryUtilities.tryGetDeployedTrajectory("ExampleTrajectory");
        var pathController = GenericAutonUtilities.createDefaultControllerBuilder()
        .withTrajectory(trajectory)
        .buildController();
        var exampleAutonomousCommand = 
        new InstantCommand(() -> swerve.resetPose(trajectory.getInitialPose().getTranslation()), swerve)
        .andThen(new OdometricSwerve_AdvancedFollowTrajectoryCommand(
            swerve,
            pathController));

        SmartDashboard.putData("Example Autonomous Command", exampleAutonomousCommand);
        SmartDashboard.putData("Barrel Racing", createAutonavBarrelRacingCommand());
    }

    private void configureSmartDashboardControls(){
        SmartDashboard.putData("Recalibrate Swerve Pose", new InstantCommand(() -> swerve.resetPose(), swerve));
        SmartDashboard.putData("Recalibrate Swerve Position", new InstantCommand(() -> swerve.resetPose(new Translation2d())));
    }
    private CommandBase createAutonavBarrelRacingCommand(){
        var traj2 = tryGetDeployedTrajectory("BarrelRacing");
        return new InstantCommand(() -> swerve.resetPose(traj2.getInitialPose().getTranslation()), swerve).andThen(new OdometricSwerve_AdvancedFollowTrajectoryCommand(
            swerve,
            GenericAutonUtilities.createDefaultControllerBuilder()
            .withEndRotation(new Rotation2d(0.0))
            .withTrajectory(tryGetDeployedTrajectory("BarrelRacing"))
            .withMaxVelocity(1)
            .buildController()));
    }

}
