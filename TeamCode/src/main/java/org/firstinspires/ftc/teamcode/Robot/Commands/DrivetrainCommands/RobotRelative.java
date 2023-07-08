package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;


import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Input;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;
import org.firstinspires.ftc.teamcode.Utils.MathUtils;

public class RobotRelative extends Command {


	protected boolean isBoostAppropriate = false;

	BasicPID heading_controller = new BasicPID(new PIDCoefficients(3,0,0.2));
	AngleController heading_control = new AngleController(heading_controller);

	Drivetrain drivetrain;
	HorizontalExtension extension;
	VerticalExtension verticalExtension;
	Input game_pad1;
	double strafe_dead_band = 0.1;

	double snap_angle = Math.toRadians(-90);

	public RobotRelative(Robot robot, Input game_pad1) {
		super(robot.drivetrain, game_pad1);
		this.drivetrain = robot.drivetrain;
		this.game_pad1 = game_pad1;
		this.extension = robot.scoringMechanism.horizontalExtension;
		this.verticalExtension = robot.scoringMechanism.verticalExtension;

	}

	@Override
	public void init() {

	}

	@Override
	public void periodic() {


		double scalar = 1.0;

		double x;
		double y;
		double turn;
		y = game_pad1.getStrafeJoystick();
		x = game_pad1.getForwardJoystick();
		turn = game_pad1.getTurnJoystick();

		if (this.extension.getSlidePosition() > 100) {
			x *= 0.4;
			y *= 0.4;
			turn *= 0.4;
		}
//
//		if (game_pad1.getLeft_trigger_value() > 0.5) {
//			turn = heading_control.calculate(
//					snap_angle,
//					drivetrain.drive.getPoseEstimate().getHeading()
//			);
//		}

		y = MathUtils.applyDeadBand(y, strafe_dead_band);

		Pose2d powers = new Pose2d(x * scalar, y * scalar, turn * scalar * 0.5);


		drivetrain.robotRelative(powers);


	}

	@Override
	public boolean completed() {
		return false;
	}

	@Override
	public void shutdown() {
		drivetrain.shutdown();
	}
}
