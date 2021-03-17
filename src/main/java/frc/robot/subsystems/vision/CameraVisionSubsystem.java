package frc.robot.subsystems.vision;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.hardware.CameraVisionComponent;

/**
 * Old vision subsystem component, using network tables and a RasPi
 * @deprecated
 * Deprecated, use {@link #VisionSubsystemNew}
 */




@Deprecated
public class CameraVisionSubsystem extends SubsystemBase {

    private CameraVisionComponent vision;
    public CameraVisionSubsystem(CameraVisionComponent vision) {
        this.vision = vision;

    }


    public double getHorizontalOffset(){
        return vision.getHorizontalOffsetFromCrosshair();
    }
    public double getVerticalOffset(){
        return vision.getVerticalOffsetFromCrosshair();
    }
    public double getAngleX(){
        return vision.getAngleX();
    }

}

