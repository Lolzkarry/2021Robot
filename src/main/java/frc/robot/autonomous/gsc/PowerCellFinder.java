package frc.robot.autonomous.gsc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.LowLightPipeline;
import frc.robot.subsystems.vision.RandomGripPipeline;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;

import java.util.ArrayList;

public class PowerCellFinder {
    int width = 320, height = 240;
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
    CvSink cvSink = CameraServer.getInstance().getVideo(camera);
    CvSource outputStream1 = CameraServer.getInstance().putVideo("Pipeline Output", width, height), outputStream2 = CameraServer.getInstance().putVideo("Captured Frame", width, height);

    Mat source = new Mat();

    LowLightPipeline pipeline = new LowLightPipeline();
    public PowerCellFinder(){
        camera.setResolution(width, height);
    }

    public ArrayList<Rect> findPowerCells() {
        if (cvSink.grabFrame(source, 10) != 0) {
            pipeline.process(source);
            outputStream2.putFrame(source);
        } else {
            System.err.println(cvSink.getError());
        }
        return pipeline.getRects();
    }
}
