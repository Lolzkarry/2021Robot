package frc.robot.autonomous;


import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.command.AdvancedSwerveController;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_AdvancedFollowTrajectoryCommand;

import static frc.robot.autonomous.ExtendedTrajectoryUtilities.tryGetDeployedTrajectory;

public class BouncePathCommand extends SequentialCommandGroup {
    private AdvancedSwerveController controller;
    public BouncePathCommand(OdometricSwerve swerve) {
        addRequirements(swerve);
        AdvancedSwerveController controllerPart1 = new AdvancedSwerveController(0.1, 0.1, true, 0.1, true, 3, 0, new Rotation2d(),2.4,tryGetDeployedTrajectory("BouncePathCommandComponent1").getStates().toArray(Trajectory.State[]::new));
        AdvancedSwerveController controllerPart2 = new AdvancedSwerveController(0.1, 0.1, true, 0.1, true, 3, 0, new Rotation2d(),2.4,tryGetDeployedTrajectory("BouncePathCommandComponent2").getStates().toArray(Trajectory.State[]::new));
        AdvancedSwerveController controllerPart3 = new AdvancedSwerveController(0.1, 0.1, true, 0.1, true, 3, 0, new Rotation2d(),2.4,tryGetDeployedTrajectory("BouncePathCommandComponent3").getStates().toArray(Trajectory.State[]::new));
        AdvancedSwerveController controllerPart4 = new AdvancedSwerveController(0.1, 0.1, true, 0.1, true, 3, 0, new Rotation2d(),2.4,tryGetDeployedTrajectory("BouncePathCommandComponent4").getStates().toArray(Trajectory.State[]::new));
        OdometricSwerve_AdvancedFollowTrajectoryCommand pathPart1 = new OdometricSwerve_AdvancedFollowTrajectoryCommand(swerve, controllerPart1);
        OdometricSwerve_AdvancedFollowTrajectoryCommand pathPart2 = new OdometricSwerve_AdvancedFollowTrajectoryCommand(swerve, controllerPart2);
        OdometricSwerve_AdvancedFollowTrajectoryCommand pathPart3 = new OdometricSwerve_AdvancedFollowTrajectoryCommand(swerve, controllerPart3);
        OdometricSwerve_AdvancedFollowTrajectoryCommand pathPart4 = new OdometricSwerve_AdvancedFollowTrajectoryCommand(swerve, controllerPart4);

        controllerPart2.setDesiredRotationOffset(Math.PI);  //commands to reverse rotation direction for the second and fourth parts of the path, to avoid unnecessary rotation
        controllerPart4.setDesiredRotationOffset(Math.PI);

        addCommands(pathPart1, pathPart2, pathPart3, pathPart4);
    }
}