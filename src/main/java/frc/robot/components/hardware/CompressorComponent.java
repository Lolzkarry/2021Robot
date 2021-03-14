/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.hardware;

import edu.wpi.first.wpilibj.Compressor;

/**
 * An {@link frc.robot.components.CompressorComponent} wrapper for {@link Compressor}.
 */
public class CompressorComponent extends Compressor implements frc.robot.components.CompressorComponent {
    /**
     * @see Compressor#Compressor(int)
     */
    public CompressorComponent(int module) {
        super(module);
    }
}
