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
    private String[][] paths;
    private HashMap<Character, Integer> arguments;
    public GalacticSearchCommand(OdometricSwerve swerve, Intake intake, Indexer indexer, Arm arm, char path, char color) { //Command to run galactic search path, path takes either a or b as argument and color takes r or b
        addRequirements(swerve, intake, indexer, arm);

        arguments.put('R', 0);   //maps arguments for A, B, Red, Blue, to path names
        arguments.put('r', 0);   //also Path B and color Blue both map to 1 that's cool
        arguments.put('B', 1);
        arguments.put('b', 1);
        arguments.put('A', 0);
        arguments.put('a', 0);
        paths[0][0] = "GSearchARed";   //TODO: add the actual filenames
        paths[0][1] = "GSearchABlue";
        paths[1][0] = "GSearchBRed";
        paths[1][1] = "GSearchBBlue";

        Command intakeCommand = new Autonomous_IndexBallsCommand(indexer, intake, 1, 0.9);
        Command setArm = new InstantCommand(() ->  arm.setAngle(Math.PI/2));
        Command resetArm = new InstantCommand(() -> arm.setAngle(0));

        try{
            Trajectory trajectory = tryGetDeployedTrajectory(paths[arguments.get(path)][arguments.get(color)]);
            InstantCommand resetPoseCommand = new InstantCommand(() -> swerve.resetPose(new Translation2d(trajectory.getInitialPose().getX(), trajectory.getInitialPose().getY())));
            AdvancedSwerveController controller = new AdvancedSwerveController(0.1, 0.1, false, 0.1, true, 3, 0, new Rotation2d(),2.4,trajectory.getStates().toArray(Trajectory.State[]::new));
            pathCommand = new OdometricSwerve_AdvancedFollowTrajectoryCommand(swerve, controller);
            addCommands(setArm, intakeCommand, resetPoseCommand, pathCommand.andThen(resetArm));
        }
        catch(IllegalArgumentException exception){
            DriverStation.reportError("Invalid argument given to GalacticSearch command", exception.getStackTrace());
        }
    }
}