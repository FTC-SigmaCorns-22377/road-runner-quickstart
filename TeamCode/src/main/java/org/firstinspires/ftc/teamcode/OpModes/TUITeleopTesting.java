package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseTeleop;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RobotRelative;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MoveFieldCursor;
import org.firstinspires.ftc.teamcode.Utils.VirtualField;
@Disabled

public class TUITeleopTesting extends BaseTeleop {

	VirtualField virtualField;

	@Override
	public Command setupTeleop(CommandScheduler scheduler) {
		virtualField = new VirtualField(telemetry);
		robot.gamepad2.whenDPadUpPressed(new MoveFieldCursor(0, -1, virtualField));
		robot.gamepad2.whenDPadDownPressed(new MoveFieldCursor(0, 1, virtualField));
		robot.gamepad2.whenDPadLeftPressed(new MoveFieldCursor(-1, 0, virtualField));
		robot.gamepad2.whenDPadRightPressed(new MoveFieldCursor(1, 0, virtualField));

		return new RobotRelative(robot, robot.gamepad1);

	}
}
