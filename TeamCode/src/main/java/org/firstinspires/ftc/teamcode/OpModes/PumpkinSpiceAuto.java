package org.firstinspires.ftc.teamcode.OpModes;

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
public class PumpkinSpiceAuto extends BaseAuto {


	final Pose2d goToPole1 = new Pose2d(-38, 24, Math.toRadians(-100));
	final Pose2d parkRight = new Pose2d(-63, 20, Math.toRadians(0));
	final Pose2d parkMID = new Pose2d(-40, 18, Math.toRadians(-90));
	//    final Pose2d parkLeft1 = new Pose2d(-38,26,Math.toRadians(-90));
//    final Pose2d parkLeft = new Pose2d(-6,38,Math.toRadians(180));
	final Pose2d parkLeft1_new = new Pose2d(-38, 19, Math.toRadians(270));
	final Pose2d parkLeft_new = new Pose2d(-8, 15, Math.toRadians(90));
	Pose2d startPose = new Pose2d(-36, 66.5, Math.toRadians(-90));
	Pose2d goToPole2 = shiftRobotRelative(
			new Pose2d(-36.2, 10.158013549498268, Math.toRadians(338.11832672430523)),
			0.6,
			-1
	);
	Pose2d goToPoleAfterCorrection = new Pose2d(goToPole2.getX(), goToPole2.getY() + 3, goToPole2.getHeading());
	final Pose2d parkRight1 = new Pose2d(goToPole2.getX() - 1, goToPole2.getY() + 3, goToPole2.getHeading());
	Pose2d DislodgePosition = shiftRobotRelative(goToPole2, -2,10);
	double backup = -2;
	Pose2d newPose = shiftRobotRelative(goToPole2,backup,0);
	Trajectory backupFromPole;

	Pose2d newPoseIfMisfired = shiftRobotRelative(goToPole2,-backup,0);
	Trajectory moveUpToPole;


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

//		 scheduler.addIntent(new ToggleBrake(robot.drivetrain).interruptOthers().when(() -> robot.drivetrain.getVelocity().getX() == 0));

		Trajectory driveToPole = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
				.splineTo(goToPole1.vec(), goToPole1.getHeading())
				.splineToSplineHeading(goToPole2, calculateTangent(goToPole1, goToPole2))
				.build();

		Trajectory parkRightTraj = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2, true)
				.splineToConstantHeading(parkRight1.vec(), calculateTangent(goToPole2, parkRight1))
				.splineToSplineHeading(parkRight, calculateTangent(parkRight1, parkRight))
				.build();

		Trajectory parkMidTraj = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2, true)
				.lineToLinearHeading(parkMID)
				.build();
//
//        Trajectory parkLeftTraj = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2,true)
//                .splineTo(parkLeft1.vec(),Math.toRadians(75))
//                .splineTo(parkLeft.vec(), Math.toRadians(0))
//                .build();

		Trajectory parkLeftTrajNew = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2, true)
				.splineToConstantHeading(parkLeft1_new.vec(), Math.toRadians(0))
				.splineToSplineHeading(parkLeft_new, Math.toRadians(0))
				.build();

		backupFromPole = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2, false)
				.lineToLinearHeading(newPose)
				.build();
		moveUpToPole = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2, false)
				.lineToLinearHeading(newPoseIfMisfired)
				.build();


		Trajectory park = parkLeftTrajNew;

		switch (parkingPosition) {
			case LEFT:
				park = parkLeftTrajNew;
				break;
			case CENTER:
				park = parkMidTraj;
				break;
			case RIGHT:
				park = parkRightTraj;
				break;
		}

		Command auto = followRR(driveToPole).addNext(new SetDrivetrainBrake(robot.drivetrain, Drivetrain.BrakeStates.FREE));


		auto.addNext(new RoadrunnerHoldPose(robot, goToPole2));
		auto.addNext(new SetDrivetrainBrake(robot.drivetrain, Drivetrain.BrakeStates.ACTIVATED));
		for (int i = 0; i < 5; ++i) {
			addCycle(auto, commandGroups);
		}

		auto.addNext(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION))
				.addNext(new Delay(0.25))
				.addNext(commandGroups.depositCone());

		auto.addNext(commandGroups.moveHorizontalExtension(0));
		auto.addNext(new SetDrivetrainBrake(robot.drivetrain, Drivetrain.BrakeStates.FREE));
		auto.addNext(new Delay(0.1));
		auto.addNext(followRR(park));
		return auto;
	}

	public void addCycle(Command command, ScoringCommandGroups commandGroups) {


		Command nextCommand = multiCommand(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION),
				commandGroups.moveToIntakingRightAuto(),
				commandGroups.moveHorizontalExtension(HorizontalExtension.PRE_EMPTIVE_EXTEND))
				.addNext(commandGroups.moveHorizontalExtension(HorizontalExtension.mostlyAutoExtension))
				.addNext(commandGroups.collectConeAutoPT1(HorizontalExtension.autoExtension,
						verticalExtensionHitPoleProcedure(commandGroups)))
				.addNext(DepositIfMisFired(commandGroups))
				.addNext(commandGroups.collectConeAutoPT1_5())
				.addNext(DislodgeConeIdeal())
				.addNext(commandGroups.collectConeAutoPT2());

		command.addNext(nextCommand);

	}

	/**
	 * if our vertical extension is hitting the pole, we want to run this,
	 * @return safety enforcing command
	 */
	public Command verticalExtensionHitPoleProcedure(ScoringCommandGroups commandGroups) {

		return new PrintCommand1(robot.print, "vertical safety initialization")
				.addNext(commandGroups.moveHorizontalExtension(VerticalExtension.HIGH_POSITION))
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
					.addNext(new SetDrivetrainBrake(robot.drivetrain, Drivetrain.BrakeStates.ACTIVATED));
			}
			System.out.println("no maneuver, null command returned");
			return new NullCommand();
		}
		);

		return c;
	}

	public Command DepositIfMisFired(ScoringCommandGroups commandGroups) {
		Command c = new RunCommand(() -> {
			System.out.println("checking if misfire occurred");
			if (robot.scoringMechanism.verticalExtension.coneIsStillInDeposit()) {
				System.out.println("Misfire did occur");
				return commandGroups.openClaw()
						.addNext(new SetDrivetrainBrake(robot.drivetrain, Drivetrain.BrakeStates.FREE))
						.addNext(new Delay(0.2))
						.addNext(followRR(moveUpToPole))
						.addNext(new SetDrivetrainBrake(robot.drivetrain, Drivetrain.BrakeStates.ACTIVATED))
						.addNext(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION))
						.addNext(commandGroups.depositConeAsync())
						.addNext(commandGroups.grabCone());
			} else {
				System.out.println("Misfire did not occur; distance was: " + robot.scoringMechanism.verticalExtension.getDistanceToDeposit() + " slide height was " + robot.scoringMechanism.verticalExtension.getSlidePosition());
				return new NullCommand();
			}
		});
		return c;
	}

}
