package org.firstinspires.ftc.teamcode.OpModes.CRI_OPMODES;

import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RobotRelative;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Input;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;

public class RobotRelativeOutreach extends RobotRelative {
	public RobotRelativeOutreach(Robot robot, Input game_pad1) {
		super(robot, game_pad1);
		scalar = 0.4;
	}
}
