package org.firstinspires.ftc.teamcode.OpModes.CRI_OPMODES;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseTeleop;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.Brake.ToggleBrake;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RobotRelative;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.NullCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.RunCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.RunCommandLegacy;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;
import org.firstinspires.ftc.teamcode.VisionUtils.VisionMode;


public class Teleop extends BaseTeleop {


	@Override
	public Command setupTeleop(CommandScheduler scheduler) {

		ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain, robot.backCamera);

		robot.gamepad1.whenDPadDownPressed(new RunCommandLegacy(commandGroups::moveToIntakingLeft));
		robot.gamepad1.whenDPadUpPressed(new RunCommandLegacy(commandGroups::moveToIntakingLeftLonger));
		robot.gamepad1.whenLeftBumperPressed(commandGroups.cornerScore());
		robot.gamepad1.whenLeftStickButtonPressed(new ToggleBrake(robot.drivetrain));
		robot.gamepad1.whenRightBumperPressed(new RunCommandLegacy(commandGroups::collectCone));
		robot.gamepad1.whenRightTriggerPressed(new RunCommandLegacy(commandGroups::moveVerticalExtensionDownOrReleaseClaw));
		robot.gamepad1.whenSquarePressed(commandGroups.moveVerticalExtension(VerticalExtension.MID_POSITION_teleop));
		robot.gamepad1.whenCrossPressed(new RunCommandLegacy(commandGroups::moveToIntakingLeftWithDeposit));
		robot.gamepad1.whenTrianglePressed(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION_teleop));
		robot.gamepad1.whenCirclePressed(commandGroups.moveToLowGoalScoring());
		robot.gamepad1.whenRightStickButtonPressed(new RunCommand(() -> {
			robot.drivetrain.setPose(new Pose2d(0,0,Math.toRadians(-90)));
			return new NullCommand();
		}));
		robot.gamepad1.whenLeftTriggerPressed(new RunCommandLegacy(commandGroups::moveToIntakingLeftLonger));
		robot.backCamera.setVisionMode(VisionMode.LION); //TODO this might be redundant, remove this if pipeline instance is created at the start of every opmode
		return new MultipleCommand(new RobotRelative(robot, robot.gamepad1));
	}
	@Override
	public VisionMode getVisionMode() { return VisionMode.LION; }
}
