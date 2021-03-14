/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.virtual;

import frc.robot.components.AngleGetterComponent;
import frc.robot.components.AngleSetterComponent;

/**
 * A virtual component which implements {@link AngleGetterComponent} and
 * {@link AngleSetterComponent}.
 */
public class VirtualAngleComponent implements AngleGetterComponent, AngleSetterComponent {

    private double angle;

    @Override
    public void setAngle(double angle) {
        this.angle = angle;
    }

    @Override
    public double getAngle() {
        return angle;
    }

}
