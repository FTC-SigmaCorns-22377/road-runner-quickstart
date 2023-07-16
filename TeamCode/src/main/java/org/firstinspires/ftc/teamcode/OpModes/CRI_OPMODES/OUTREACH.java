package org.firstinspires.ftc.teamcode.OpModes.CRI_OPMODES;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseTeleop;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;

@TeleOp
public class OUTREACH extends BaseTeleop {
	@Override
	public Command setupTeleop(CommandScheduler scheduler) {
		return new RobotRelativeOutreach(robot,robot.gamepad1);
	}
}
