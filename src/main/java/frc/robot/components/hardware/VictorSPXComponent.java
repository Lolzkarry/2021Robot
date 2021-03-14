/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.components.OutputSetterComponent;

/**
 * An {@link OutputSetterComponent} wrapper for {@link VictorSPX}.
 */
public class VictorSPXComponent extends VictorSPX implements OutputSetterComponent {

    /**
     * @see VictorSPX#VictorSPX(int)
     */
    public VictorSPXComponent(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public void setOutput(double speed) {
        set(ControlMode.PercentOutput, -speed);
    }

}
