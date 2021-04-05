// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.gsc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import org.opencv.core.Point;


/** Add your docs here. */
public class ConfigurationIdentifier {
    private HashMap<GalacticSearchConfiguration, Point[]> configurationPoints;
    public ConfigurationIdentifier(){
        configurationPoints = new HashMap<>();

        configurationPoints.put(GalacticSearchConfiguration.ARed, new Point[]{
            new Point(0, 0),
            new Point(0,0),
            new Point(0,0)
        });

        configurationPoints.put(GalacticSearchConfiguration.ABlue, new Point[]{
            new Point(0,0),
            new Point(),
            new Point()
        });

        configurationPoints.put(GalacticSearchConfiguration.BRed, new Point[]{
            new Point(),
            new Point(),
            new Point()
        });
        
        configurationPoints.put(GalacticSearchConfiguration.BBlue, new Point[]{
            new Point(),
            new Point(),
            new Point()
        });
    } 
    public GalacticSearchConfiguration identifyConfiguration(Point[] cells){
        //Points, in this bit of code, refers to the calibrated location of a power cell
        //Cell refers to the location of the power cell detected by the camera
        HashMap<GalacticSearchConfiguration, Double> totalDistances = new HashMap<>();
        for(var entry : configurationPoints.entrySet()){
            ArrayList<Point> pointsTemp = new ArrayList<>(Arrays.asList(entry.getValue()));
            for(var cell : cells){
                ArrayList<Double> distances = new ArrayList<>();
                for(var point : pointsTemp){
                    double currentDistance = findSquareDist(point, cell);
                    distances.add(currentDistance);
                }

                totalDistances.put(entry.getKey(), totalDistances.getOrDefault(entry.getKey(), 0.0) + Collections.min(distances));
                pointsTemp.remove(distances.indexOf(Collections.min(distances)));
            }
        }
        return Collections.min(totalDistances.entrySet(),Map.Entry.comparingByValue()).getKey();
    }
    private double findSquareDist(Point p1, Point p2){ //finds pixel distance in terms of the area of a rectangle between the points
        double diffX = Math.abs(p1.x-p2.x);
        double diffY = Math.abs(p1.y-p2.y);
        double product = diffX * diffY;
        return product;
    }
}
