package frc.robot.autonomous;


import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.command.AdvancedSwerveController;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_AdvancedFollowTrajectoryCommand;

import java.time.Instant;

import static frc.robot.autonomous.ExtendedTrajectoryUtilities.*;

public class BouncePathCommand extends SequentialCommandGroup {
    private AdvancedSwerveController controller;
    public BouncePathCommand(OdometricSwerve swerve) {
        addRequirements(swerve);
        InstantCommand resetPose = new InstantCommand(() ->  swerve.resetPose(new Translation2d(0.7752273295375641, 2.349173725871407)));

        Command pathPart1 = addDottedTrajectoryWithShuffleboard(swerve, "Bounce Path1", "BouncePathComponent1", true, 0);
        Command pathPart2 = addDottedTrajectoryWithShuffleboard(swerve, "Bounce Path2", "BouncePathComponent2", true, Math.PI);
        Command pathPart3 = addDottedTrajectoryWithShuffleboard(swerve, "Bounce Path3", "BouncePathComponent3", true, 0);
        Command pathPart4 = addDottedTrajectoryWithShuffleboard(swerve, "Bounce Path4", "BouncePathComponent4", true, Math.PI);

        addCommands(resetPose, pathPart1, pathPart2, pathPart3, pathPart4);
    }
}