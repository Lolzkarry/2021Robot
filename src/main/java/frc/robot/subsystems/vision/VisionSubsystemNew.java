package frc.robot.subsystems.vision;


import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.opencv.core.Mat;

public class VisionSubsystemNew extends SubsystemBase {
    private CvSink sink;
    private CvSource out;
    public VisionSubsystemNew() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        CameraServer.getInstance().startAutomaticCapture();
        sink = CameraServer.getInstance().getVideo();
        out = (CameraServer.getInstance().putVideo("cam", 640, 480));
    }

    public Mat getFrame(){
    }
}

