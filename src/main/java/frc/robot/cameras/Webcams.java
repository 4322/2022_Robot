package frc.robot.cameras;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import frc.robot.Constants;

public class Webcams {

    private UsbCamera leftCamera;
    private UsbCamera rightCamera;
    private UsbCamera frontCamera;

    public Webcams() {
        leftCamera = CameraServer.startAutomaticCapture(0);
        rightCamera = CameraServer.startAutomaticCapture(1);
        frontCamera = CameraServer.startAutomaticCapture(2);

        leftCamera.setResolution(320, 240);
        rightCamera.setResolution(320, 240);
        frontCamera.setResolution(320, 240);

        leftCamera.setFPS(Constants.WebCams.fps);
        rightCamera.setFPS(Constants.WebCams.fps);
        frontCamera.setFPS(Constants.WebCams.fps);

        leftCamera.setPixelFormat(VideoMode.PixelFormat.kYUYV);
        rightCamera.setPixelFormat(VideoMode.PixelFormat.kYUYV);
        frontCamera.setPixelFormat(VideoMode.PixelFormat.kYUYV);

        resetCameras();
    }

    public void setFront() {
        rightCamera.setExposureManual(Constants.WebCams.darkExposure);
        leftCamera.setExposureManual(Constants.WebCams.darkExposure);
        frontCamera.setExposureAuto();
    }

    public void setLeft() {
        rightCamera.setExposureManual(Constants.WebCams.darkExposure);
        leftCamera.setExposureAuto();
        frontCamera.setExposureManual(Constants.WebCams.darkExposure);
    }

    public void setRight() {
        rightCamera.setExposureAuto();
        leftCamera.setExposureManual(Constants.WebCams.darkExposure);
        frontCamera.setExposureManual(Constants.WebCams.darkExposure);
    }

    public void resetCameras() {
        rightCamera.setExposureAuto();
        leftCamera.setExposureAuto();
        frontCamera.setExposureAuto();
    }
}