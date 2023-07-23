package org.firstinspires.ftc.teamcode.OpModes.CRI_OPMODES;

import static org.firstinspires.ftc.teamcode.RR_quickstart.util.BasedMath.shiftRobotRelative;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.RR_quickstart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.Brake.SetDrivetrainBrake;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RoadrunnerHoldPose;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.NullCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.RunCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveHorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.SetSlideWeaken;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;
import org.firstinspires.ftc.teamcode.Simulation.TestCommandsSubsystems.PrintCommand1;
import org.firstinspires.ftc.teamcode.Utils.Team;

@Autonomous
public class LeftHighPole extends BaseAuto {


	final Pose2d goToPole1 = new Pose2d(-38, -24, Math.toRadians(-100));
	final Pose2d parkRight = new Pose2d(-63, -20, Math.toRadians(0));
	final Pose2d parkMID = new Pose2d(-40, -18, Math.toRadians(90));
	final Pose2d parkLeft1_new = new Pose2d(-38, -19, Math.toRadians(270));
	final Pose2d parkLeft_new = new Pose2d(-6, -17, Math.toRadians(90));
	Pose2d startPose = new Pose2d(-36, -66.5, Math.toRadians(90));
	Pose2d goToPole2 = shiftRobotRelative(
			new Pose2d(-36.95, -6.5, -Math.toRadians(338.11832672430523 - 3)),
			-2.45,
			-1.7
	);
	final Pose2d parkLefter1 = new Pose2d(0, -22, -Math.toRadians(270));
	final Pose2d parkLefter_new = new Pose2d(15, -19,-Math.toRadians(90));

	Pose2d goToPoleAfterCorrection = new Pose2d(goToPole2.getX(), goToPole2.getY() - 3, goToPole2.getHeading());
	final Pose2d parkRight1 = new Pose2d(goToPole2.getX() - 1, goToPole2.getY() - 3, goToPole2.getHeading());
	Pose2d DislodgePosition = shiftRobotRelative(goToPole2, -2,10);
	double backup = -2;
	Pose2d newPose = shiftRobotRelative(goToPole2,backup,0);

	Trajectory backupFromPole;

	Pose2d newPoseIfMisfired = shiftRobotRelative(goToPole2,-backup,0);
	Trajectory moveUpToPole;
	Trajectory park;
	int cycle_count = 0;


	@Override
	public Team getTeam() {
		return Team.BLUE;
	}

	@Override
	public void setRobotPosition() {
		robot.drivetrain.setPose(startPose);
	}

	@Override
	public Command setupAuto(CommandScheduler scheduler) {
		ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain, robot.backCamera);


		Trajectory driveToPole = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
				.splineTo(goToPole1.vec(), -goToPole1.getHeading())
				.splineToSplineHeading(goToPole2,calculateTangent(goToPole1,goToPole2))
				.build();

		Trajectory parkMidTraj = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2, true)
				.lineToLinearHeading(parkMID)
				.build();


		Trajectory parkRightTraj = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2, true)
				.splineToSplineHeading(parkRight, calculateTangent(parkRight,parkRight))
				.build();

		Trajectory parkLeftTrajNew = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2, true)
				.splineToConstantHeading(parkLeft1_new.vec(), Math.toRadians(0))
				.splineToSplineHeading(parkLeft_new, Math.toRadians(0))
				.build();
		Trajectory parkLefter = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2, true)
				.splineToConstantHeading(parkLefter1.vec(), Math.toRadians(0))
				.splineToSplineHeading(parkLefter_new, Math.toRadians(0))
				.build();


		backupFromPole = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2, false)
				.lineToLinearHeading(newPose)
				.build();
		moveUpToPole = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2, false)
				.lineToLinearHeading(newPoseIfMisfired)
				.build();


		park = parkLeftTrajNew;

		switch (parkingPosition) {
			case ZONE_1:
				park = parkRightTraj;
				break;
			case ZONE_2:
				park = parkMidTraj;
				break;
			case ZONE_3:
				park = parkLeftTrajNew;
				break;
		}


		Command auto = followRR(driveToPole).addNext(new SetDrivetrainBrake(robot.drivetrain, Drivetrain.BrakeStates.FREE));


		auto.addNext(new RoadrunnerHoldPose(robot, goToPole2));
		auto.addNext(new SetDrivetrainBrake(robot.drivetrain, Drivetrain.BrakeStates.ACTIVATED));
		for (int i = 0; i < 5; ++i) {
			addCycle(auto, commandGroups);
		}
		auto.addNext(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION_LEFT_AUTO))
				.addNext(new Delay(0.25))
				.addNext(commandGroups.depositCone());
		auto.addNext(commandGroups.moveHorizontalExtension(0));
		auto.addNext(new SetDrivetrainBrake(robot.drivetrain, Drivetrain.BrakeStates.FREE));
		auto.addNext(new Delay(0.1));
		auto.addNext(followRR(park));
		return auto;
	}

	public void addCycle(Command command, ScoringCommandGroups commandGroups) {


		Command nextCommand = multiCommand(
				commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION_LEFT_AUTO),
				commandGroups.moveToIntakingLeftAuto()
		)// 0.45 s
				.addNext(commandGroups.depositConeAsync())
				.addNext(commandGroups.moveHorizontalExtension(HorizontalExtension.mostlyAutoExtensionLeft))
				.addNext(commandGroups.depositConeAndGrabCone(HorizontalExtension.autoExtensionLeft))
				.addNext(DepositIfMisFired(commandGroups))
				.addNext(commandGroups.bringConeIn())
				.addNext(DislodgeConeIdeal())
				.addNext(commandGroups.completeConeTransfer());

		command.addNext(nextCommand);

	}

	/**
	 * if our vertical extension is hitting the pole, we want to run this,
	 * @return safety enforcing command
	 */
	public Command verticalExtensionHitPoleProcedure(ScoringCommandGroups commandGroups) {

		return new PrintCommand1(robot.print, "vertical safety initialization")
				.addNext(commandGroups.moveHorizontalExtension(VerticalExtension.HIGH_POSITION_LEFT_AUTO))
				.addNext(followRR(backupFromPole))
				.addNext(commandGroups.asyncMoveVerticalExtension(VerticalExtension.IN_POSITION));

	}



	public Command DislodgeConeIdeal() {

		Trajectory DislodgeCone1 = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2,false)
				.lineToLinearHeading(DislodgePosition, SampleMecanumDrive.VEL_CONSTRAINT_FAST_TURN, SampleMecanumDrive.ACCEL_CONSTRAINT)
				.build();

		DislodgePosition = new Pose2d(DislodgePosition.getX(), DislodgePosition.getY(), Math.toRadians(-90));

		Trajectory DislodgeCone2 = robot.drivetrain.getBuilder().trajectoryBuilder(DislodgePosition,false)
				.lineToLinearHeading(goToPoleAfterCorrection)
				.build();

		Command c = new RunCommand(() -> {
			System.out.println("Current at evaluation is " + robot.scoringMechanism.horizontalExtension.getCurrent());
			if (robot.scoringMechanism.horizontalExtension.currentExceedsCutoff()) {
				System.out.println("maneuver occurring");
				double slidePosition = robot.scoringMechanism.horizontalExtension.getSlidePosition();
				return new SetDrivetrainBrake(robot.drivetrain, Drivetrain.BrakeStates.FREE)
						.addNext(new Delay(0.1))
						.addNext(new SetSlideWeaken(robot.scoringMechanism.horizontalExtension, true))
						.addNext(new MoveHorizontalExtension(robot.scoringMechanism.horizontalExtension, slidePosition))
						.addNext(followRR(DislodgeCone1))
						.addNext(new SetSlideWeaken(robot.scoringMechanism.horizontalExtension, false))
						.addNext(new MoveHorizontalExtension(robot.scoringMechanism.horizontalExtension, HorizontalExtension.DISLODGE_CONE))
						.addNext(followRR(DislodgeCone2))
						.addNext(new MoveHorizontalExtension(robot.scoringMechanism.horizontalExtension,HorizontalExtension.IN_POSITION))
						.addNext(new Delay(0.1))
						.addNext(followRR(park))
						.addNext(new Delay(30));
			}
			System.out.println("no maneuver, null command returned");
			return new NullCommand();
		}
		);

		return c;
	}

	public Command DepositIfMisFired(ScoringCommandGroups commandGroups) {
		return new RunCommand(() -> {
			cycle_count += 1;
			boolean cone_in_deposit = robot.scoringMechanism.verticalExtension.coneIsStillInDeposit();
			System.out.println("checking if misfire occurred");
			if (cone_in_deposit && cycle_count == 1) {
				System.out.println("Misfire did occur");

				return commandGroups.openClaw()
						.addNext(new SetDrivetrainBrake(robot.drivetrain, Drivetrain.BrakeStates.FREE))
						.addNext(new Delay(0.2))
						.addNext(followRR(moveUpToPole))
						.addNext(new SetDrivetrainBrake(robot.drivetrain, Drivetrain.BrakeStates.ACTIVATED))
						.addNext(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION_RIGHT_AUTO))
						.addNext(commandGroups.depositConeAsync())
						.addNext(commandGroups.grabCone());
			} else if (cone_in_deposit) {
				return commandGroups.openClaw()
						.addNext(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION_RIGHT_AUTO))
						.addNext(commandGroups.depositConeAsync())
						.addNext(commandGroups.grabCone());
			} else  {
				System.out.println("Misfire did not occur; distance was: " + robot.scoringMechanism.verticalExtension.getDistanceToDeposit() + " slide height was " + robot.scoringMechanism.verticalExtension.getSlidePosition());
				return new NullCommand();
			}
		});
	}

}
