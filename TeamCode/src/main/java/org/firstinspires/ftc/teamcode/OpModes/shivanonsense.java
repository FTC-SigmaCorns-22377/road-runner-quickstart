package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Utils.Team;
@Disabled

public class shivanonsense extends BaseAuto {
	@Override
	public Command setupAuto(CommandScheduler scheduler) {
		Trajectory scoring = robot.drivetrain.getBuilder().trajectoryBuilder(new Pose2d(-52, 0)).lineToSplineHeading(new Pose2d(-26, 0, Math.toRadians(180)))
				.lineToSplineHeading(new Pose2d(0, 0, Math.toRadians(0)))
				.lineToSplineHeading(new Pose2d(26, 0, Math.toRadians(180)))
				.lineToSplineHeading(new Pose2d(52, 0, Math.toRadians(0)))
				.build();

		return followRR(robot.drivetrain.getBuilder().trajectoryBuilder(new Pose2d(-52, 0)).lineToSplineHeading(new Pose2d(-26, 0, Math.toRadians(180)))
				.build()).addNext(followRR(robot.drivetrain.getBuilder().trajectoryBuilder(new Pose2d(-52, 0)).lineToSplineHeading(new Pose2d(0, 0, Math.toRadians(0)))
				.build())).addNext(followRR(robot.drivetrain.getBuilder().trajectoryBuilder(new Pose2d(-52, 0)).lineToSplineHeading(new Pose2d(26, 0, Math.toRadians(180)))
				.build())).addNext(followRR(robot.drivetrain.getBuilder().trajectoryBuilder(new Pose2d(-52, 0)).lineToSplineHeading(new Pose2d(52, 0, Math.toRadians(0)))
				.build()));


	}

	@Override
	public Team getTeam() {
		return Team.BLUE;
	}

	private enum ParkingPosition {
		LEFT,
		RIGHT,
		CENTER
	}
//    public void addCycle(Command command, ScoringCommandGroups commandGroups) {
//        command.addNext(multiCommand(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION),
//                        commandGroups.moveToIntakingRightAuto(),
//                        commandGroups.moveHorizontalExtension(HorizontalExtension.PRE_EMPTIVE_EXTEND)))
//                .addNext(commandGroups.moveHorizontalExtension(HorizontalExtension.mostlyAutoExtension))
//                .addNext(commandGroups.collectConeAuto());
//    }
}
