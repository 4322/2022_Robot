package frc.robot.cameras;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Webcams {

    private ShuffleboardTab tab;

    private UsbCamera leftCamera;
    private UsbCamera rightCamera;

    public Webcams() {
        tab = Shuffleboard.getTab("Webcams");

        leftCamera = CameraServer.startAutomaticCapture(0);
        rightCamera = CameraServer.startAutomaticCapture(1);

        leftCamera.setFPS(10);
        leftCamera.setResolution(320, 240);
        rightCamera.setFPS(10);
        rightCamera.setResolution(320, 240);

        leftCamera.setPixelFormat(VideoMode.PixelFormat.kYUYV);
        rightCamera.setPixelFormat(VideoMode.PixelFormat.kYUYV);
    }

    public void setLeft() {
        leftCamera = CameraServer.startAutomaticCapture(0);
        rightCamera.close();
        leftCamera.setFPS(20);
    }

    public void setRight() {
        rightCamera = CameraServer.startAutomaticCapture(1);
        leftCamera.close();
        rightCamera.setFPS(20);
    }

    public void resetCameras() {
        leftCamera = CameraServer.startAutomaticCapture(0);
        rightCamera = CameraServer.startAutomaticCapture(1);

        leftCamera.setFPS(10);
        rightCamera.setFPS(10);
    }
}