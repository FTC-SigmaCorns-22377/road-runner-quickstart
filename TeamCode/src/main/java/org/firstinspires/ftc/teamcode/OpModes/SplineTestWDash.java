package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
@Disabled

public class SplineTestWDash extends BaseAuto {
	@Override
	public Command setupAuto(CommandScheduler scheduler) {
		Trajectory traj = robot.drivetrain.getBuilder().trajectoryBuilder(new Pose2d())
				.splineTo(new Vector2d(30, 30), 0)
				.build();
		Trajectory traj2 = robot.drivetrain.getBuilder().trajectoryBuilder(traj.end(), true)
				.splineTo(new Vector2d(0, 0), Math.toRadians(180))
				.build();
		return followRR(traj)
				.addNext(followRR(traj2));
	}
}
