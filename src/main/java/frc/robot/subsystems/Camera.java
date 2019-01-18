/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Camera extends Subsystem implements Runnable {
  Thread cameraThread;
  
  UsbCamera camera;
  CvSink cvSink;
  CvSource outputStream;
  Mat source;

  public Camera() {
    // Run camera code in separate thread to avoid
    // interfering with other code.
    cameraThread = new Thread(this);
    
    // Automatically find camera (in any USB port)
    camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(640, 480);
    
    cvSink = CameraServer.getInstance().getVideo();
    outputStream = CameraServer.getInstance().putVideo("Vision", 640, 480);

    source = new Mat();
  }

  // run() is automatically run inside cameraThread
  public void run() {
    // This loop runs forever, but will pause
    // automatically so other code can run:
    while (!Thread.interrupted()) {
      cvSink.grabFrame(source);
      outputStream.putFrame(source);
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
