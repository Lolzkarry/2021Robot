package frc.robot.autonomous;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.command.AdvancedSwerveController;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_AdvancedFollowTrajectoryCommand;

import java.io.IOException;
import java.time.Instant;
import java.util.HashMap;

import static frc.robot.autonomous.ExtendedTrajectoryUtilities.tryGetDeployedTrajectory;

public class GalacticSearchCommand extends ParallelCommandGroup { //TODO: Create another command to identify which path to take
    private OdometricSwerve_AdvancedFollowTrajectoryCommand pathCommand;
    private HashMap<String, String> paths;
    public GalacticSearchCommand(OdometricSwerve swerve, Intake intake, Indexer indexer, Arm arm, String path) { //Command to run galactic search path, path takes either a or b as argument and color takes r or b
        addRequirements(swerve, intake, indexer, arm);

        paths.put("ARed", "GSearchARed");//TODO: add the actual filenames
        paths.put("ABlue", "GSearchABlue");
        paths.put("BRed", "GSearchBRed");
        paths.put("BBlue", "GSearchBBlue");

        //commands to make robot do
        Command intakeCommand = new Autonomous_IndexBallsCommand(indexer, intake, 1, 0.9);
        Command setArm = new InstantCommand(() ->  arm.setAngle(Math.PI/2));
        Command resetArm = new InstantCommand(() -> arm.setAngle(0));

        //gets trajectory to run
        String trajectoryStr = paths.get(path);
        if (trajectoryStr == null) { //check if path string maps to a path
            DriverStation.reportError("Invalid path string given to galactic search command", true);
            throw new RuntimeException("Invalid path string given to galactic search command");
        }
        Trajectory trajectory = tryGetDeployedTrajectory(trajectoryStr);

        //aligns robot to path and constructs path command
        InstantCommand resetPoseCommand = new InstantCommand(() -> swerve.resetPose(new Translation2d(trajectory.getInitialPose().getX(), trajectory.getInitialPose().getY())));
        AdvancedSwerveController controller = new AdvancedSwerveController(0.1, 0.1, false, 0.1, true, 3, 0, new Rotation2d(),2.4,trajectory.getStates().toArray(Trajectory.State[]::new));
        pathCommand = new OdometricSwerve_AdvancedFollowTrajectoryCommand(swerve, controller);



        addCommands(setArm, intakeCommand, resetPoseCommand, pathCommand.andThen(resetArm));
        
    }
}