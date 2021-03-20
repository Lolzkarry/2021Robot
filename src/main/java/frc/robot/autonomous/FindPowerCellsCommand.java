package frc.robot.autonomous;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.vision.RandomGripPipeline;

public class FindPowerCellsCommand extends InstantCommand{
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
    CvSink cvSink = CameraServer.getInstance().getVideo(camera);
    CvSource outputStream1 = CameraServer.getInstance().putVideo("Pipeline Output", -1, -1), outputStream2 = CameraServer.getInstance().putVideo("Captured Frame", -1, -1);
    
  Mat source = new Mat();

  RandomGripPipeline pipeline = new RandomGripPipeline();

  @Override
  public void execute() {
    if(cvSink.grabFrame(source, 10) != 0){
        pipeline.process(source);
        outputStream2.putFrame(source);
    }else{
        System.err.println(cvSink.getError());
    }
    if(!pipeline.findContoursOutput().isEmpty()){
        outputStream1.putFrame(pipeline.findContoursOutput().get(0));

        SmartDashboard.putStringArray("Contour Outputs", pipeline.findContoursOutput().stream().map( wtf -> {
            return wtf.toString();
        }).toArray(String[]::new));
    }
  }
}