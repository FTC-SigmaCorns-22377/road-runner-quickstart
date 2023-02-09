package org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Utils.Team;
import org.firstinspires.ftc.teamcode.VisionUtils.Cone;
import org.firstinspires.ftc.teamcode.VisionUtils.Resolution;
import org.firstinspires.ftc.teamcode.visionPipelines.ConeDetectionFast;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class BackCamera extends Subsystem {
    public Size resolution = Resolution.LOW;
    public Size nativeResolution = new Size(1920,1080);
    public double HFOV = Math.toRadians(67.8727791718758);//68.67
    public double VFOV = Math.toRadians(41.473850212095506);//42.07
//    public double HFOV = Math.toRadians(67.9255464975384);
//    public double VFOV = Math.toRadians(41.50936520321168);

    public Pose2d position = new Pose2d(0,0, Math.toRadians(0));
    private OpenCvPipeline pipeline;
    private OpenCvWebcam cam;
    private final OpenCvCameraRotation cameraRotation = OpenCvCameraRotation.UPRIGHT;
    private final ExposureControl.Mode exposureMode = ExposureControl.Mode.Manual;
    private final long exposureMs = 40;
    private final int gain = 100;
    private final FocusControl.Mode focusMode = FocusControl.Mode.Fixed;
    private final double focusLength = 69; //idk what units this is in
    private List<Cone> tempConeList;

    public BackCamera(Team team) {
//        pipeline = new Save(team,this);
        pipeline = new ConeDetectionFast(team,this);
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        cam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Back Webcam"), hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName()));
        cam.setViewportRenderer(OpenCvWebcam.ViewportRenderer.GPU_ACCELERATED);
        cam.setPipeline(pipeline);
        cam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                cam.startStreaming( (int) resolution.width, (int) resolution.height, cameraRotation);
                cam.getExposureControl().setMode(exposureMode);
                cam.getExposureControl().setExposure(exposureMs, TimeUnit.MILLISECONDS);
                cam.getGainControl().setGain(gain);
                cam.getFocusControl().setMode(focusMode);
                if (focusMode == FocusControl.Mode.Fixed) {
                    cam.getFocusControl().setFocusLength(focusLength);
                }
            }
            @Override public void onError(int errorCode) {
                shutdown();
            }});
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(cam, 20);
    }

    @Override
    public void periodic() { Dashboard.packet.put("Back FPS", cam.getFps()); }

    @Override
    public void shutdown() { cam.closeCameraDevice(); }

    @Nullable
    public Cone getCone() {
        assert pipeline instanceof ConeDetectionFast;
        tempConeList = ((ConeDetectionFast) pipeline).getCones();
        if (tempConeList.size() > 0) return tempConeList.get(0);
        return null;
    }
    @Nullable
    public Cone getCone(int rank) {
        assert pipeline instanceof ConeDetectionFast;
        tempConeList = ((ConeDetectionFast) pipeline).getCones();
        if (tempConeList.size() >= rank) return tempConeList.get(rank);
        return null;
    }

    @Nullable
    public Cone getConeStack() {
        assert pipeline instanceof ConeDetectionFast;
        return ((ConeDetectionFast) pipeline).conestackGuess;
    }

}
