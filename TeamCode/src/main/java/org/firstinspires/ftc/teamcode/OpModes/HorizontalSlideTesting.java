package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Utils.Team;
@Disabled

public class HorizontalSlideTesting extends BaseAuto {
	@Override
	public Command setupAuto(CommandScheduler scheduler) {
		ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain, robot.backCamera);
		waitForStart();
		return commandGroups.moveHorizontalExtension(ManualHorizontalSlide.position);
	}
	//0: 2
	//50: 3.5
	//100: 5
	//150: 6.5 (slightly under)
	//200: 8
	//250: 9.5
	//300: 11

	@Override
	public Team getTeam() {
		return Team.RED;
	}

	@Config
	public static class ManualHorizontalSlide {
		public static double position = 0;
	}
}
