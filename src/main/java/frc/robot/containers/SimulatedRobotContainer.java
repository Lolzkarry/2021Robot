// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.containers;

import static frc.robot.autonomous.ExtendedTrajectoryUtilities.tryGetDeployedTrajectory;
import static frc.robot.autonomous.GenericAutonUtilities.createDefaultControllerBuilder;
import static frc.robot.utility.ExtendedMath.withHardDeadzone;
import static frc.robot.utility.ExtendedMath.withContinuousDeadzone;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.autonomous.BouncePathCommand;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.OdometricSwerveDashboardUtility;
import frc.robot.subsystems.swerve.odometric.command.AdvancedSwerveControllerDashboardUtility;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_AdvancedFollowTrajectoryCommand;
import frc.robot.subsystems.swerve.odometric.factory.OdometricSimulatedSwerveFactory;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SimulatedRobotContainer implements IRobotContainer {
    XboxController controller = new XboxController(0);
    OdometricSwerve swerve = new OdometricSimulatedSwerveFactory().makeSwerve();

    public SimulatedRobotContainer() {
        swerve.setDefaultCommand(
                new RunCommand(() -> swerve.moveRobotCentric(withDeadzone(controller.getX(Hand.kLeft), 0.2) * 10,
                        -withDeadzone(controller.getY(Hand.kLeft), 0.2) * 10,
                        withDeadzone(controller.getX(Hand.kRight), 0.2) * 3), swerve));

        var ac = createDefaultControllerBuilder().withTrajectory(tryGetDeployedTrajectory("ExampleTrajectory"))
                .buildController();
        SmartDashboard.putData("Follow Trajectory", new InstantCommand(() -> swerve.resetPose(new Translation2d(0.762, 12.714)), swerve).andThen(new OdometricSwerve_AdvancedFollowTrajectoryCommand(swerve, ac)));
        SmartDashboard.putData("Bounce Path", new BouncePathCommand(swerve));
        SmartDashboard.putData("Reset Pose", new InstantCommand(() -> swerve.resetPose(new Pose2d()),swerve));
        SmartDashboard.putData("Swerve Transform", new OdometricSwerveDashboardUtility(swerve));
        SmartDashboard.putData("Current State Transform", new AdvancedSwerveControllerDashboardUtility(ac));
        SmartDashboard.putData("bounce path", new BouncePathCommand(swerve));
    }

    private double withDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0;
        } else {
            return value;
        }
    }
}
