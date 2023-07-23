package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RoadrunnerHoldPose;

@Autonomous
public class PoseHoldTest extends BaseAuto {


	@Override
	public void setRobotPosition() {
		robot.drivetrain.setPose(new Pose2d(0,0,0));
	}

	@Override
	public Command setupAuto(CommandScheduler scheduler) {
		return new RoadrunnerHoldPose(robot, new Pose2d(0,0,0));
	}
}
