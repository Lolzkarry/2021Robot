// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.pshoot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.autonomous.VisionDistanceCalculator;

/** Add your docs here. */
public class PreciserVisionPreciseShootingOI extends VisionPreciseShootingOI implements Sendable{

    DoubleSupplier supplier;

    double maxCoefficient = 1.2;
    double minCoefficient = 1;

    public PreciserVisionPreciseShootingOI(VisionDistanceCalculator visionDistanceCalculator, DoubleSupplier supplier) {
        super(visionDistanceCalculator);
        this.supplier = supplier;
    }
    @Override
    public double getCoefficient() {
        return supplier.getAsDouble() * (maxCoefficient - minCoefficient) + minCoefficient;
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Current Coefficient", this::getCoefficient, null);
        builder.addDoubleProperty("Min Coefficient", this::getMinCoefficient, this::setMinCoefficient);
        builder.addDoubleProperty("Max Coefficient", this::getMaxCoefficient, this::setMaxCoefficient);

    }
    public double getMaxCoefficient() {
        return maxCoefficient;
    }
    public void setMaxCoefficient(double maxCoefficient) {
        this.maxCoefficient = maxCoefficient;
    }
    public double getMinCoefficient() {
        return minCoefficient;
    }
    public void setMinCoefficient(double minCoefficient) {
        this.minCoefficient = minCoefficient;
    }
    
}
    

