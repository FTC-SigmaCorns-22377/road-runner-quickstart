package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.Brake.SetDrivetrainBrake;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.NullCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.RunCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.AsyncMoveVerticalExtension;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.CloseLatch;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveArm;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveArmDirect;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveArmIfCone;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveClaw;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveClawOnceSlidesDown;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveHorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveHorizontalExtensionAsync;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveTurret;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveTurretAsync;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveVerticalExtension;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveVerticalExtensionAsync;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveVerticalExtensionTeleop;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.OpenLatch;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.SetHorizontalExtensionInches;
import org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands.CancelableMoveArmDirect;
import org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands.GetIntakeParameters;
import org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands.VisualIntakeStage1;
import org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands.VisualIntakeStage2;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.MainScoringMechanism;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.BackCamera;
import org.firstinspires.ftc.teamcode.VisionUtils.IntakeParameters;

public class ScoringCommandGroups {
	public int currentCone = 5;
	BackCamera backCamera;
	Turret turret;
	VerticalExtension verticalExtension;
	HorizontalExtension horizontalExtension;
	Drivetrain drivetrain;
	Pose2d intakePosition = new Pose2d();
	double[] armConeHeights = {0.08, 0.11, 0.1357, 0.1632, 0.1800};

	double[] armConeHeightsVision = {0.0699, 0.11, 0.1357, 0.1502, 0.1670};
	boolean hasAlreadyRanTeleAuto = false;

	public ScoringCommandGroups(MainScoringMechanism mechanism, Drivetrain drivetrain, BackCamera backCamera) {
		this.horizontalExtension = mechanism.horizontalExtension;
		this.verticalExtension = mechanism.verticalExtension;
		this.turret = mechanism.turret;
		this.drivetrain = drivetrain;
		this.backCamera = backCamera;
	}

	public Command cornerScore(){
		return moveArm(Turret.ArmStates.LOW_SCORING).addNext(new SetHorizontalExtensionInches(horizontalExtension, 16));
	}

	public Command fastTeleAutoSingleMeasure() {
		GetIntakeParameters getIntakeParameters = new GetIntakeParameters(turret, backCamera, horizontalExtension);
		IntakeParameters intakeParameters = getIntakeParameters.getIntakeParameters();
		Command command = new NullCommand()
				.addNext(openLatch())
				.addNext(new SetDrivetrainBrake(drivetrain, Drivetrain.BrakeStates.ACTIVATED))
				.addNext(getIntakeParameters)
				.addNext(new VisualIntakeStage1(intakeParameters, turret,horizontalExtension))
				.addNext(setArmHeightVisionStack(intakeParameters))
				.addNext(new VisualIntakeStage2(intakeParameters, turret,horizontalExtension))
				.addNext(new Delay(0.1))
				.addNext(grabCone())


				.addNext(moveArmIfCone(Turret.ArmStates.TRANSFER_SAFE, intakeParameters))
				.addNext(moveTurret(Turret.TurretStates.TRANSFER))
				.addNext(moveHorizontalExtension(HorizontalExtension.IN_POSITION))
				.addNext(moveArmIfCone(Turret.ArmStates.TRANSFER,intakeParameters))
				.addNext(new Delay(0.1))
				.addNext(releaseCone())
				.addNext(closeLatch())
				.addNext(moveArmIfCone(Turret.ArmStates.TRANSFER_SAFE,intakeParameters));


		for (int i = 0; i < 2; ++i) {
			command.addNext(moveVerticalExtension(VerticalExtension.HIGH_POSITION + .06))
					.addNext(new MultipleCommand(
							new VisualIntakeStage1(intakeParameters, turret,horizontalExtension)
									.addNext(setArmHeightVisionStack(intakeParameters))
							,
							depositConeAsync()
					))
					.addNext(new VisualIntakeStage2(intakeParameters, turret,horizontalExtension))
					.addNext(new Delay(0.1))
					.addNext(grabCone())
					.addNext(new Delay(0.1))

					.addNext(moveArmIfCone(Turret.ArmStates.TRANSFER_SAFE, intakeParameters))
					.addNext(moveTurret(Turret.TurretStates.TRANSFER))
					.addNext(moveHorizontalExtension(HorizontalExtension.IN_POSITION))
					.addNext(moveArmIfCone(Turret.ArmStates.TRANSFER, intakeParameters))

					.addNext(new Delay(0.1))
					.addNext(releaseCone())
					.addNext(closeLatch())
					.addNext(new Delay(0.1))
					.addNext(moveArmIfCone(Turret.ArmStates.TRANSFER_SAFE, intakeParameters));


		}
		command.addNext(moveVerticalExtension(VerticalExtension.HIGH_POSITION + .06))
				.addNext(new MultipleCommand(
						new VisualIntakeStage1(intakeParameters, turret,horizontalExtension)
								.addNext(setArmHeightVisionStack(intakeParameters))
						,
						depositConeAsync()
				))
				.addNext(new VisualIntakeStage2(intakeParameters, turret,horizontalExtension))
				.addNext(new Delay(0.1))
				.addNext(grabCone())
				.addNext(new Delay(0.1))

				.addNext(moveArmIfCone(Turret.ArmStates.TRANSFER_SAFE, intakeParameters))
				.addNext(moveHorizontalExtension(HorizontalExtension.IN_POSITION))
				.addNext(moveArmIfCone(Turret.ArmStates.LOW_SCORING, intakeParameters));
		return command
				.addNext(new SetDrivetrainBrake(drivetrain, Drivetrain.BrakeStates.FREE));
	}
	public Command fastTeleAuto() {

		if (hasAlreadyRanTeleAuto) {
			return new NullCommand();
		}

		hasAlreadyRanTeleAuto = true;

		GetIntakeParameters getIntakeParameters = new GetIntakeParameters(turret, backCamera, horizontalExtension);
		IntakeParameters intakeParameters = getIntakeParameters.getIntakeParameters();
		Command command = new NullCommand()
				.addNext(new SetDrivetrainBrake(drivetrain, Drivetrain.BrakeStates.ACTIVATED))
				.addNext(getIntakeParameters)//new MultipleCommand(getIntakeParameters,openLatch())
				.addNext(new VisualIntakeStage1(intakeParameters, turret,horizontalExtension))
				.addNext(setArmHeightVisionStack(intakeParameters))
				.addNext(new VisualIntakeStage2(intakeParameters, turret,horizontalExtension))
				.addNext(new Delay(0.1))
				.addNext(grabCone())


				.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE))
				.addNext(moveTurret(Turret.TurretStates.TRANSFER))
				.addNext(moveHorizontalExtension(HorizontalExtension.IN_POSITION))
				.addNext(moveArm(Turret.ArmStates.TRANSFER))


				.addNext(new Delay(0.1))
				.addNext(releaseCone())
				.addNext(closeLatch())
				.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE));



		for (int i = 0; i < 4; ++i) {
			GetIntakeParameters getCycleIntakeParameters = new GetIntakeParameters(turret, backCamera, horizontalExtension);
			IntakeParameters cycleIntakeParameters = getCycleIntakeParameters.getIntakeParameters();
			command.addNext(getCycleIntakeParameters)//new MultipleCommand(getCycleIntakeParameters,openLatch()))
					.addNext(moveVerticalExtension(VerticalExtension.HIGH_POSITION + .06))
					.addNext(new MultipleCommand(
							new VisualIntakeStage1(cycleIntakeParameters, turret,horizontalExtension)
									.addNext(setArmHeightVisionStack(cycleIntakeParameters))
							,
							depositConeAsync()
					))
					.addNext(new VisualIntakeStage2(cycleIntakeParameters, turret,horizontalExtension))
					.addNext(new Delay(0.1))
					.addNext(grabCone())
					.addNext(new Delay(0.1))


					.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE))
					.addNext(moveTurret(Turret.TurretStates.TRANSFER))
					.addNext(moveHorizontalExtension(HorizontalExtension.IN_POSITION))
					.addNext(moveArm(Turret.ArmStates.TRANSFER))


					.addNext(new Delay(0.1))
					.addNext(releaseCone())
					.addNext(closeLatch())
					.addNext(new Delay(0.1))
					.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE));
		}
		return command
				.addNext(new SetDrivetrainBrake(drivetrain, Drivetrain.BrakeStates.FREE));
	}



	public Command teleAuto() {
		Command command = new NullCommand();

		for (int i = 0; i < 4; ++i) {
			GetIntakeParameters getCycleIntakeParameters = new GetIntakeParameters(turret, backCamera, horizontalExtension);
			IntakeParameters cycleIntakeParameters = getCycleIntakeParameters.getIntakeParameters();
			command.addNext(getCycleIntakeParameters)
					.addNext(new VisualIntakeStage1(cycleIntakeParameters, turret,horizontalExtension))
					.addNext(setArmHeightVisionStack(cycleIntakeParameters))
					.addNext(new VisualIntakeStage2(cycleIntakeParameters, turret,horizontalExtension))
					.addNext(new Delay(0.1))
					.addNext(grabCone())
					.addNext(new Delay(0.1))
					.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE))
					.addNext(moveTurret(Turret.TurretStates.TRANSFER))
					.addNext(moveHorizontalExtension(HorizontalExtension.IN_POSITION))
					.addNext(moveArm(Turret.ArmStates.TRANSFER))
					.addNext(new Delay(0.1))
					.addNext(releaseCone())
					.addNext(closeLatch())
					.addNext(new Delay(0.1))
					.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE))
					.addNext(moveVerticalExtension(VerticalExtension.HIGH_POSITION + .06))
					.addNext(depositConeAsync())
					.addNext(openLatch())
					.addNext(new Delay(0.1));
		}
		GetIntakeParameters getIntakeParameters = new GetIntakeParameters(turret, backCamera, horizontalExtension);
		IntakeParameters intakeParameters = getIntakeParameters.getIntakeParameters();
		return command.addNext(getIntakeParameters)
				.addNext(new VisualIntakeStage1(intakeParameters, turret,horizontalExtension))
				.addNext(setArmHeightVisionStack(intakeParameters))
				.addNext(new VisualIntakeStage2(intakeParameters, turret,horizontalExtension))
				.addNext(new Delay(0.1))
				.addNext(grabCone())
				.addNext(new Delay(0.1))
				.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE))
				.addNext(moveTurret(Turret.TurretStates.TRANSFER))
				.addNext(moveHorizontalExtension(HorizontalExtension.IN_POSITION))
				.addNext(moveArm(Turret.ArmStates.TRANSFER))
				.addNext(new Delay(0.1))
				.addNext(releaseCone())
				.addNext(closeLatch())
				.addNext(new Delay(0.1))
				.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE));
	}
//	public Command autoGoToCone() {
//		return new RunCommand(() -> {
//			Cone cone = backCamera.getCone();
//			if (cone != null) {
//
//				double angle = IntakeKinematics.getTurretAngleToTarget(-1 * cone.position.dx);
//				double extendDistance = IntakeKinematics.getHorizontalSlideExtensionToTarget(cone.position.dy, -1 * cone.position.dx, horizontalExtension.getSlidePositionInches());
//				Dashboard.packet.put("THE dx", cone.position.dx);
//				Dashboard.packet.put("THE dy", cone.position.dy);
//				Dashboard.packet.put("THE angle", Math.toDegrees(angle));
//				Dashboard.packet.put("THE extendDist", extendDistance);
//				if (extendDistance <= 15) {
//					if (angle < 0) {
//						angle += Math.PI * 2;
//					}
//					return openClaw().addNext(new MoveTurretDirect(turret, angle).addNext(new SetHorizontalExtensionInches(horizontalExtension, extendDistance)).addNext(moveArm(Turret.ArmStates.DOWN)));
//				} else return new NullCommand();
//
//			} else return new NullCommand();
//		});
//	}
	public Command autoGoToCone() {
		GetIntakeParameters getIntakeParameters = new GetIntakeParameters(turret, backCamera, horizontalExtension);
		IntakeParameters intakeParameters = getIntakeParameters.getIntakeParameters();
		return getIntakeParameters
				.addNext(new VisualIntakeStage1(intakeParameters, turret,horizontalExtension))
				.addNext(new MultipleCommand(moveArm(Turret.ArmStates.DOWN), openClaw()))
				.addNext(new VisualIntakeStage2(intakeParameters, turret,horizontalExtension));
	}
	public Command autoIntake() {
		return autoGoToCone()
				.addNext(new Delay(0.1))
				.addNext(grabCone())
				.addNext(new Delay(0.1))
				.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE))
				.addNext(moveTurret(Turret.TurretStates.TRANSFER))
				.addNext(moveHorizontalExtension(HorizontalExtension.IN_POSITION))
				.addNext(moveArm(Turret.ArmStates.TRANSFER))
				.addNext(new Delay(0.1))
				.addNext(releaseCone())
				.addNext(closeLatch());
	}


	public Command setArmHeightVisionStack(IntakeParameters intakeParameters) {
		currentCone--;
		return new MultipleCommand(new CancelableMoveArmDirect(intakeParameters,turret,-0.02 + armConeHeightsVision[currentCone]), openClaw());
	}


	// near straight but tilted to the left
	public Command moveToIntakingLeft() {
		return moveHorizontalExtension(HorizontalExtension.CLOSE_INTAKE)
				.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE))
				.addNext(moveTurret(Turret.TurretStates.Slight_LEFT))
				.addNext(new Delay(0.1))
				.addNext(new MultipleCommand(moveArm(Turret.ArmStates.DOWN), openClaw()));
	}
	public Command moveToIntakingLeftLonger() {
		return new MultipleCommand(moveHorizontalExtension(HorizontalExtension.autoExtension), moveArm(Turret.ArmStates.TRANSFER_SAFE), moveTurret(Turret.TurretStates.Slight_LEFT))
				.addNext(new Delay(0.1))
				.addNext(new MultipleCommand(moveArm(Turret.ArmStates.DOWN), openClaw()));
	}

	public Command moveToIntakingLeftWithDeposit() {
		Pose2d drivetrainPos = drivetrain.getPose();
		if (Math.hypot(intakePosition.getX() - drivetrainPos.getX(), intakePosition.getY() - drivetrainPos.getY()) > 15) {
			// if this is the case, dont put out the extension, just put out the vertical extension
			return moveVerticalExtension(VerticalExtension.HIGH_POSITION);
		}
		return new MoveHorizontalExtensionAsync(horizontalExtension,HorizontalExtension.CLOSE_INTAKE)	//new MultipleCommand(moveHorizontalExtension(HorizontalExtension.CLOSE_INTAKE), moveVerticalExtension(VerticalExtension.HIGH_POSITION))
				.addNext(new MoveVerticalExtensionAsync(verticalExtension,VerticalExtension.HIGH_POSITION,drivetrain))
				.addNext(moveTurret(Turret.TurretStates.Slight_LEFT))
				.addNext(new MultipleCommand(moveArm(Turret.ArmStates.DOWN), openClaw()));
	}

	public Command moveToIntakingRight() {
		return moveHorizontalExtension(HorizontalExtension.EXTENSION1)
				.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE))
				.addNext(moveTurret(Turret.TurretStates.Slight_RIGHT_AUTO))
				.addNext(new MultipleCommand(moveArm(Turret.ArmStates.DOWN), openClaw()));
	}

	public Command moveToIntakingPosition() {
		currentCone--;

		return moveArm(Turret.ArmStates.TRANSFER_SAFE)
				.addNext(moveTurret(Turret.TurretStates.Slight_RIGHT_AUTO))
				.addNext(new MultipleCommand(moveArmDirect(-0.04 + armConeHeights[currentCone]), openClaw()));
	}

	public Command moveToIntakingLeftAuto() {
		// 0.45 s total
		currentCone--;

		return moveArm(Turret.ArmStates.TRANSFER_SAFE)// 0.15 s
				.addNext(moveTurret(Turret.TurretStates.Slight_LEFT_AUTO)) // 0.15 s
				.addNext(new MultipleCommand(moveArmDirect(-0.04 + armConeHeights[currentCone]), openClaw())); // 0.15s
	}

	public Command moveToIntakingLeftSideMidAuto() {
		currentCone--;

		return moveArm(Turret.ArmStates.TRANSFER_SAFE)
				.addNext(moveTurret(Turret.TurretStates.Slight_RIGHT_AUTO))
				.addNext(new MultipleCommand(moveArmDirect(-0.02 + armConeHeights[currentCone]), openClaw()));
	}

	// very far to the left, in order to place near by
	public Command moveToIntakingLeftClosePole() {
		return moveArm(Turret.ArmStates.TRANSFER_SAFE)
				.addNext(moveHorizontalExtension(HorizontalExtension.PRE_EMPTIVE_EXTEND))
				.addNext(moveTurret(Turret.TurretStates.Slight_LEFT))//new MultipleCommand(moveArm(Turret.ArmStates.TRANSFER_SAFE),
				//moveHorizontalExtension(HorizontalExtension.PRE_EMPTIVE_EXTEND),
				//moveTurret(Turret.TurretStates.FAR_LEFT))
				.addNext(new MultipleCommand(moveArm(Turret.ArmStates.DOWN), openClaw()))
				.addNext(moveHorizontalExtension(HorizontalExtension.TELE_CYCLE_EXTENSION));
	}

	public Command moveToIntakingRightClosePole() {
		Command command = new MultipleCommand(moveArm(Turret.ArmStates.TRANSFER_SAFE),
				moveHorizontalExtension(HorizontalExtension.PRE_EMPTIVE_EXTEND),
				moveTurret(Turret.TurretStates.Slight_RIGHT))
				.addNext(new Delay(0.1))
				.addNext(new MultipleCommand(moveArm(Turret.ArmStates.DOWN), openClaw()))
				.addNext(moveHorizontalExtension(HorizontalExtension.TELE_CYCLE_EXTENSION));
		return new RunCommand(() -> command);
	}

	public Command syncedScoringCollecting() {
		Command depositCommand = moveVerticalExtension(VerticalExtension.HIGH_POSITION);
		Command intakeCommand = moveToIntakingLeft();
		return intakeCommand.addNext(depositCommand);
	}


	public Command collectCone() {

		intakePosition = drivetrain.getPose();

		if (verticalExtension.getSlidePosition() > 50) {
			return new NullCommand();
		}

		return grabCone() // 0.25s
				.addNext(openLatch()) // 0s
				.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE)) // 0.25s
				.addNext(new Delay(0.15))
				.addNext(moveTurret(Turret.TurretStates.TRANSFER)) // 0.2s
				.addNext(new Delay(0.15)) // 0.15s
				.addNext(moveHorizontalExtension(HorizontalExtension.IN_POSITION)) // some amount of time, assuming about 0.2
				.addNext(moveArm(Turret.ArmStates.TRANSFER)) // 0.25s
				.addNext(new Delay(0.15)) // 0.15s
				.addNext(closeLatch()) // 0.0s
				.addNext(new Delay(0.15))
				.addNext(releaseCone())  // 0.25s
				.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE)); // 0.25
	}

	public Command collectConeAuto(double autoExtensionDistance) {

		return depositConeAsync()
				.addNext(openLatch())
				.addNext(moveHorizontalExtension(autoExtensionDistance))
				.addNext(new Delay(0.25))
				.addNext(grabCone())
				.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE))
				.addNext(moveTurret(Turret.TurretStates.TRANSFER))
				.addNext(moveHorizontalExtension(HorizontalExtension.IN_POSITION))
				.addNext(moveArm(Turret.ArmStates.TRANSFER))
				.addNext(new Delay(0.15))
				.addNext(releaseCone())
				.addNext(closeLatch())
				.addNext(new Delay(0.1))
				.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE));

	}



	public Command depositConeAndGrabCone(double autoExtensionDistance) {

		if (currentCone == 0) {
			autoExtensionDistance += 10;
		}

		return depositConeAsync() // 0
				.addNext(moveHorizontalExtension(autoExtensionDistance))
				.addNext(new MoveClawOnceSlidesDown(turret,verticalExtension, Turret.ClawStates.Closed));
	}

	public Command bringConeIn() {
		return moveArm(Turret.ArmStates.TRANSFER_SAFE)
				.addNext(moveTurret(Turret.TurretStates.TRANSFER))
				.addNext(moveHorizontalExtension(HorizontalExtension.IN_POSITION));

	}

	public Command completeConeTransfer() {
		return moveArm(Turret.ArmStates.TRANSFER)
				.addNext(releaseCone())
				.addNext(closeLatch())
				.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE));

	}


	public Command grabCone() {
		return moveClaw(Turret.ClawStates.Closed);
	}

	public Command releaseCone() {
		return moveClaw(Turret.ClawStates.Transfer);
	}

	public Command openClaw() {
		return moveClaw(Turret.ClawStates.Open);
	}

	public Command moveToLowGoalScoring() {
		return moveClaw(Turret.ClawStates.Closed)
				.addNext(moveArm(Turret.ArmStates.LOW_SCORING))
				.addNext(moveHorizontalExtension(HorizontalExtension.IN_POSITION));
	}

	// Move vertical extension down. If it is already going down, open the claw
	public Command moveVerticalExtensionDownOrReleaseClaw() {
		if (verticalExtension.getSlideTargetPosition() == VerticalExtension.IN_POSITION) {
			return moveClaw(Turret.ClawStates.Open);
		} else {
			return new MultipleCommand(new MoveVerticalExtensionTeleop(verticalExtension,VerticalExtension.IN_POSITION)
			,
					new Delay(0.3)
							.addNext(new OpenLatch(turret)
					)
			);
		}
	}


	public Command asyncMoveVerticalExtension(double position) {
		return new AsyncMoveVerticalExtension(verticalExtension, drivetrain, position);
	}

	public MoveArm moveArm(Turret.ArmStates armStates) {
		return new MoveArm(turret, armStates);
	}
	public MoveArmIfCone moveArmIfCone(Turret.ArmStates armStates, IntakeParameters intakeParameters) {
		return new MoveArmIfCone(turret, armStates, intakeParameters);
	}

	public MoveArmDirect moveArmDirect(double position) {
		return new MoveArmDirect(turret, position);
	}

	public MoveClaw moveClaw(Turret.ClawStates clawStates) {
		return new MoveClaw(turret, clawStates);
	}

	public MoveTurret moveTurret(Turret.TurretStates turretStates) {
		return new MoveTurret(turret, turretStates);
	}

	public MoveTurret moveTurretAsync(Turret.TurretStates turretStates) {
		return new MoveTurretAsync(turret, turretStates);
	}

	public MoveHorizontalExtension moveHorizontalExtension(double position) {
		return new MoveHorizontalExtension(horizontalExtension, position);
	}

	public MoveVerticalExtension moveVerticalExtension(double position) {
		return new MoveVerticalExtension(verticalExtension, position);
	}

	public Command depositCone() {
		return openLatch().addNext(moveVerticalExtension(VerticalExtension.IN_POSITION));
	}

	public Command depositConeAsync() {
		return openLatch().addNext(asyncMoveVerticalExtension(VerticalExtension.IN_POSITION));
	}

	public CloseLatch closeLatch() {
		return new CloseLatch(turret);
	}

	public OpenLatch openLatch() {
		return new OpenLatch(turret);
	}

}
