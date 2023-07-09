package org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Math.Kinematics.IntakeKinematics;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Utils.Team;
import org.firstinspires.ftc.teamcode.VisionUtils.Cone;
import org.firstinspires.ftc.teamcode.VisionUtils.Resolution;
import org.firstinspires.ftc.teamcode.VisionUtils.VisionMode;
import org.firstinspires.ftc.teamcode.visionPipelines.ConeDetectionFast;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class BackCamera extends Subsystem {
	@Config
	public static class CamConfig {
		public static int exposureMicroSec = 60000;
	}
	private final OpenCvCameraRotation cameraRotation = OpenCvCameraRotation.UPRIGHT;
	private final ExposureControl.Mode exposureMode = ExposureControl.Mode.Manual;
//	private final long exposureMs = 30;
	public static int gain = 70;
	private final FocusControl.Mode focusMode = FocusControl.Mode.Fixed;
	private final double focusLength = 69; //idk what units this is in
	public Size resolution = Resolution.LOW;
	public Size nativeResolution = new Size(1920, 1080);
	public double HFOV = Math.toRadians(67.8727791718758);//68.67
	public double VFOV = Math.toRadians(41.473850212095506);//42.07
	public Pose2d position = new Pose2d(0, 0, Math.toRadians(0));
	private final OpenCvPipeline pipeline;
	private OpenCvWebcam cam;
	private List<Cone> tempConeList;
	public static boolean streamBackCameraToDash = false;

	public BackCamera(Team team, VisionMode visionMode) {
        //pipeline = new Save(team,this);
		pipeline = new ConeDetectionFast(team, visionMode, this);
	}

	@Override
	public void initAuto(HardwareMap hwMap) {
		if (streamBackCameraToDash) {
			cam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Back Webcam"), hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName()));
		}
		else {
			cam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Back Webcam"));
		}

		cam.setViewportRenderer(OpenCvWebcam.ViewportRenderer.GPU_ACCELERATED);
		cam.setPipeline(pipeline);
		cam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				cam.startStreaming((int) resolution.width, (int) resolution.height, cameraRotation);
				cam.getExposureControl().setMode(exposureMode);
				cam.getExposureControl().setExposure((long) CamConfig.exposureMicroSec, TimeUnit.MICROSECONDS);
				cam.getGainControl().setGain(gain);
				cam.getFocusControl().setMode(focusMode);
				if (focusMode == FocusControl.Mode.Fixed) {
					cam.getFocusControl().setFocusLength(focusLength);
				}
			}

			@Override
			public void onError(int errorCode) {
				shutdown();
			}
		});

		if (streamBackCameraToDash) { // if false, it will stream the front camera,
			FtcDashboard dashboard = FtcDashboard.getInstance();
			dashboard.startCameraStream(cam, 20);
		}

	}

	@Override
	public void periodic() {
//		cam.getExposureControl().setExposure((long) CamConfig.exposureMicroSec, TimeUnit.MILLISECONDS);
		Dashboard.packet.put("Back FPS", cam.getFps());
	}

	@Override
	public void shutdown() {
		cam.closeCameraDevice();
	}

	@Nullable
	public Cone getCone() {
		assert pipeline instanceof ConeDetectionFast;
		tempConeList = ((ConeDetectionFast) pipeline).getCones();
		if (tempConeList.size() > 0) return tempConeList.get(0);
		return null;
	}

	@Nullable
	public List<Double> getAngleAndDistance(double currentDistance) {
		assert pipeline instanceof ConeDetectionFast;
		tempConeList = ((ConeDetectionFast) pipeline).getCones();
		if (tempConeList.size() > 0) {
			double angle = IntakeKinematics.getTurretAngleToTarget(-1 * tempConeList.get(0).position.dx);
			double extendDistance = IntakeKinematics.getHorizontalSlideExtensionToTarget(tempConeList.get(0).position.dy, -1 * tempConeList.get(0).position.dx,currentDistance);
			return Arrays.asList(angle,extendDistance);
		}
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

	public void setVisionMode(VisionMode visionMode) {
		assert pipeline instanceof ConeDetectionFast;
		((ConeDetectionFast) pipeline).setVisionMode(visionMode);
	}

}
