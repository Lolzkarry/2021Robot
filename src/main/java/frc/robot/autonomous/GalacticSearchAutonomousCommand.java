package frc.robot.autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autonomous.pshoot.SmartDashboardPreciseShootingOI;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.vision.CameraVisionSubsystem;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;

/**
 * Autonomous GalacticSearch command
 * Finds the path for which the positions of balls in the camera best approximates, and runs it
 *
 * "Cells" and "Balls", and "Configurations" and "paths" are interchangeable, I can't be arsed to write consistent documentation
 *
 * MAKE SURE that points are X,Y
 *
 * Also all ints will be Integer objects because they play nicer with HashMaps
 */
public class GalacticSearchAutonomousCommand extends InstantCommand {
    private final Arm arm;
    private final Indexer indexer;
    private final Intake intake;
    private final OdometricSwerve swerve;
    HashMap<String, Integer[][]> locations; //pre-calibrated pixel locations of the cells within each configuration
    Integer[][] cells; //cell pixel locations gotten from camera
    ArrayList<String> paths; //arraylist which maps paths to numbers, ARed is 0, ABlue is 1, etc.

    public GalacticSearchAutonomousCommand(Arm arm, Indexer indexer, Intake intake, OdometricSwerve swerve) {
        this.arm = arm;
        this.indexer = indexer;
        this.intake = intake;
        this.swerve = swerve;
        addRequirements(arm, indexer, intake, swerve);

        locations.put("ARed", new Integer[][] {
                {0,0},  //pixel location of ball 1
                {0,0},  //ball 2
                {0,0}   //ball 3
        });
        locations.put("ABlue", new Integer[][] {
                {0,0},  //pixel location of ball 1
                {0,0},  //ball 2
                {0,0}   //ball 3
        });
        locations.put("BRed", new Integer[][] {
                {0,0},  //pixel location of ball 1
                {0,0},  //ball 2
                {0,0}   //ball 3
        });
        locations.put("BBlue", new Integer[][] {
                {0,0},  //pixel location of ball 1
                {0,0},  //ball 2
                {0,0}   //ball 3
        });
        cells = new Integer[][] {
                {0,0},
                {0,0},
                {0,0},
        };
        paths = new ArrayList<String>();
        paths.add("ARed"); paths.add("ABlue"); paths.add("BRed"); paths.add("BBlue");


    }

    @Override
    public void execute() {
        new FindPowerCellsCommand();
        String[] points = SmartDashboard.getStringArray("Contour Outputs", null);
        //TODO: set cells[] to whatever the GetPowerCellsCommand is outputting, and raise an error if the length isnt 3
        if (points == null) {
            DriverStation.reportError("Outputs of FindPowerCellsCommand not found in Shuffleboard", Thread.currentThread().getStackTrace());
            throw new RuntimeException("Outputs of FindPowerCellsCommand not found in Shuffleboard");
        }

        ArrayList<Integer> tDists = new ArrayList<>(); //total distances of each path
        tDists.add(0); tDists.add(0); tDists.add(0); tDists.add(0); //initialize four values to 0

        for (int path = 0 ; path < 4; path++){ //for each possible path
            ArrayList<Integer[]> pathTemp = new ArrayList<>(Arrays.asList(locations.get(paths.get(path)))); //stores a temporary arraylist of path cell coordinates
            for (int cell = 0; cell < 3; cell++){ //for each cell detected by the camera, find the closest cell in the path and strike it from the list
                Integer[] cellCoord = cells[cell]; // coordinate of the current cell
                ArrayList<Integer> distances = new ArrayList<Integer>();
                for (int pathCoordIndex = 0; pathCoordIndex < pathTemp.size(); pathCoordIndex++){ //for each coordinate in the path not assigned to a cell
                    Integer currentDistance = findSquareDist(cellCoord, pathTemp.get(pathCoordIndex));
                    distances.add(currentDistance);
                }
                tDists.set(path, tDists.get(path) + Collections.min(distances)); //adds distance for ball to total distance for the path
                pathTemp.remove(distances.indexOf(Collections.min(distances))); //removes thecoordinate values for the ball which is closest
            }
        }

        String pathToRun = paths.get(tDists.indexOf(Collections.min(tDists))); //gets the path name of the path with smallest total distance from cells to coordinates
        new GalacticSearchCommand(swerve, intake, indexer, arm, pathToRun);

    }


    private int findSquareDist(Integer[] obj1, Integer[] obj2){ //finds pixel distance in terms of the area of a rectangle between the points
        int diffX = Math.abs(obj1[0] - obj2[0]);
        int diffY = Math.abs(obj1[1] - obj2[1]);
        int product = diffX * diffY;
        return product;
    }

}
