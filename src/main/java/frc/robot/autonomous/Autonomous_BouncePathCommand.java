package frc.robot.autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.io.IOException;
import java.nio.file.Path;

public class Autonomous_BouncePathCommand extends CommandBase {
    private Trajectory trajectory;
    private String trajJSON;

    public void initialize(){
        trajectory = new Trajectory();
        trajJSON = "paths/output/BouncePath.wpilib.json";
        try{
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        }
        catch (IOException except){
            DriverStation.reportError("Unable to open trajectory " + trajJSON, except.getStackTrace());
        }
    }
}
