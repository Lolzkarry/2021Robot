package frc.robot.autonomous;


import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.command.AdvancedSwerveController;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_AdvancedFollowTrajectoryCommand;

import java.time.Instant;

import static frc.robot.autonomous.ExtendedTrajectoryUtilities.tryGetDeployedTrajectory;

public class BouncePathCommand extends SequentialCommandGroup {
    private final int samplingInterval;
    private final double speed;
    private final double kW;

    public BouncePathCommand(OdometricSwerve swerve) {
        addRequirements(swerve);
        speed = 2;
        kW = 3;
        samplingInterval = 1;

        InstantCommand resetPose = new InstantCommand(() ->  swerve.resetPose(new Translation2d(0.7752273295375641, 2.349173725871407)));
        AdvancedSwerveController controllerPart1 = new AdvancedSwerveController(0.1, 0.1, true, 0.15, true, 3, kW, new Rotation2d(),speed,tryGetDeployedTrajectory("BouncePathComponent1").getStates().toArray(Trajectory.State[]::new));
        AdvancedSwerveController controllerPart2 = new AdvancedSwerveController(0.1, 0.1, true, 0.15, true, 3, kW, new Rotation2d(),speed,tryGetDeployedTrajectory("BouncePathComponent2").getStates().toArray(Trajectory.State[]::new));
        AdvancedSwerveController controllerPart3 = new AdvancedSwerveController(0.1, 0.1, true, 0.15, true, 3, kW, new Rotation2d(),speed,tryGetDeployedTrajectory("BouncePathComponent3").getStates().toArray(Trajectory.State[]::new));
        AdvancedSwerveController controllerPart4 = new AdvancedSwerveController(0.1, 0.1, true, 0.15, true, 3, kW, new Rotation2d(),speed,tryGetDeployedTrajectory("BouncePathComponent4").getStates().toArray(Trajectory.State[]::new));

        controllerPart1.setContinuousRotation();
        controllerPart2.setContinuousRotation();
        controllerPart3.setContinuousRotation();
        controllerPart4.setContinuousRotation();
        controllerPart1.setSamplingRate(2);
        controllerPart2.setSamplingRate(2);
        controllerPart3.setSamplingRate(2);
        controllerPart4.setSamplingRate(2);



        controllerPart1.setSamplingRate(samplingInterval);
        controllerPart2.setSamplingRate(samplingInterval);
        controllerPart3.setSamplingRate(samplingInterval);
        controllerPart4.setSamplingRate(samplingInterval);

        OdometricSwerve_AdvancedFollowTrajectoryCommand pathPart1 = new OdometricSwerve_AdvancedFollowTrajectoryCommand(swerve, controllerPart1);
        OdometricSwerve_AdvancedFollowTrajectoryCommand pathPart2 = new OdometricSwerve_AdvancedFollowTrajectoryCommand(swerve, controllerPart2);
        OdometricSwerve_AdvancedFollowTrajectoryCommand pathPart3 = new OdometricSwerve_AdvancedFollowTrajectoryCommand(swerve, controllerPart3);
        OdometricSwerve_AdvancedFollowTrajectoryCommand pathPart4 = new OdometricSwerve_AdvancedFollowTrajectoryCommand(swerve, controllerPart4);

        controllerPart2.setDesiredRotationOffset(Math.PI);  //commands to reverse rotation direction for the second and fourth parts of the path, to avoid unnecessary rotation
        controllerPart4.setDesiredRotationOffset(Math.PI);

        addCommands(resetPose, pathPart1, pathPart2, pathPart3, pathPart4);
    }
}