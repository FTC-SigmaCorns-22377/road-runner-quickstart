package com.thermalequilibrium.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

public class MeepMeepTesting {
	public static double MAX_VEL = 65;
	public static double MAX_ACCEL = 65;
	public static double MAX_ANG_VEL = Math.toRadians(150);
	public static double MAX_ANG_ACCEL = Math.toRadians(120);

	public static void main(String[] args) {
		MeepMeep meepMeep = new MeepMeep(800);
		final Pose2d goToPole1 = new Pose2d(-38, -24, Math.toRadians(-100));
		final Pose2d parkRight = new Pose2d(-63, 20, Math.toRadians(0));
		final Pose2d parkMID = new Pose2d(-40, 18, Math.toRadians(-90));
		final Pose2d parkLeft1_new = new Pose2d(-38, 19, Math.toRadians(270));
		final Pose2d parkLeft_new = new Pose2d(-8, 15, Math.toRadians(90));
		Pose2d startPose = new Pose2d(-36 + 0.76, -66.5, Math.toRadians(90));
		Pose2d goToPole2 = shiftRobotRelative(
				new Pose2d(-36.2, -10.158013549498268, -Math.toRadians(338.11832672430523)),
				-1.2,
				-1.7
		);
		final Pose2d parkLefter1 = new Pose2d(0, -19, -Math.toRadians(270));
		final Pose2d parkLefter_new = new Pose2d(12, -19,-Math.toRadians(90));

		Pose2d goToPoleAfterCorrection = new Pose2d(goToPole2.getX(), goToPole2.getY() + 3, goToPole2.getHeading());
		final Pose2d parkRight1 = new Pose2d(goToPole2.getX() - 1, goToPole2.getY() + 3, goToPole2.getHeading());
		Pose2d DislodgePosition = shiftRobotRelative(goToPole2, -2,10);
		double backup = -2;
		Pose2d newPose = shiftRobotRelative(goToPole2,backup,0);
		Trajectory backupFromPole;

		Pose2d newPoseIfMisfired = shiftRobotRelative(goToPole2,-backup,0);
		Trajectory moveUpToPole;


		RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, 12)
				.followTrajectorySequence(drive ->
						drive.trajectorySequenceBuilder(startPose)
								.splineTo(goToPole1.vec(), -goToPole1.getHeading())
								.splineToSplineHeading(goToPole2,calculateTangent(goToPole1,goToPole2))
								.setReversed(true)
								.splineToConstantHeading(parkLeft1_new.vec(),Math.toRadians(0))
								.splineToSplineHeading(parkLeft_new,Math.toRadians(0))
								.build()

				);

		meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
				.setDarkMode(true)
				.setBackgroundAlpha(0.95f)
				.addEntity(myBot.setDimensions(13,17))

				.start();
	}

	public static Pose2d shiftRobotRelative(Pose2d poseGlobal, double robotRelativeX, double robotRelativeY) {
		// unit vector of the global Pose
		Vector2d globalVec = poseGlobal.vec();
		Vector2d globalVecUnit = poseGlobal.headingVec();//globalVec.div(globalVec.norm());  // vec / mag = unit vector
		// orthogonal vec
		Vector2d orthogonalVec = new Vector2d(-globalVecUnit.getY(),globalVecUnit.getX());

		Vector2d combinedVector = globalVec.plus(globalVecUnit.times(robotRelativeX))
				.plus(orthogonalVec.times(robotRelativeY));
		return new Pose2d(combinedVector, poseGlobal.getHeading());
	}
	public static double calculateTangent(Pose2d initialPosition, Pose2d finalPosition) {
		double xd = initialPosition.getX() - finalPosition.getX();
		double yd = initialPosition.getY() - finalPosition.getY();
		return Math.atan2(yd,xd) - Math.PI;
	}
}