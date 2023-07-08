package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.RR_quickstart.util.BasedMath.shiftRobotRelative;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.Brake.SetDrivetrainBrake;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.Brake.ToggleBrake;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RoadrunnerHoldPose;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands.GetIntakeParameters;
import org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands.VisualIntakeStage1;
import org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands.VisualIntakeStage2;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;
import org.firstinspires.ftc.teamcode.Utils.Team;
import org.firstinspires.ftc.teamcode.VisionUtils.IntakeParameters;
import org.firstinspires.ftc.teamcode.VisionUtils.VisionMode;

@Disabled
public class VisionSpiceRed extends BaseAuto {


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
            0.3,
            -1
    );
    final Pose2d parkRight1 = new Pose2d(goToPole2.getX() - 1, goToPole2.getY() + 3, goToPole2.getHeading());

    @Override
    public void setRobotPosition() {
        robot.drivetrain.setPose(startPose);
    }

    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain, robot.backCamera);

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

        Command auto = followRR(driveToPole);

        auto.addNext(new SetDrivetrainBrake(robot.drivetrain, Drivetrain.BrakeStates.ACTIVATED));
        auto.addNext(new RoadrunnerHoldPose(robot, goToPole2));

        GetIntakeParameters getIntakeParameters = new GetIntakeParameters(robot.scoringMechanism.turret, robot.backCamera, robot.scoringMechanism.horizontalExtension);
        IntakeParameters intakeParameters = getIntakeParameters.getIntakeParameters();
        auto.addNext(getIntakeParameters);

        for (int i = 0; i < 5; ++i) {
            addCycle(auto, commandGroups, intakeParameters);
        }

        auto.addNext(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION +.06))
                .addNext(new Delay(0.1))
                .addNext(commandGroups.depositCone());

        auto.addNext(new SetDrivetrainBrake(robot.drivetrain, Drivetrain.BrakeStates.FREE));
        auto.addNext(followRR(park));
        return auto;
    }

    public void addCycle(Command command, ScoringCommandGroups commandGroups, IntakeParameters intakeParameters) {
        command.addNext(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION + .06))
                .addNext(commandGroups.depositConeAsync())
                .addNext(commandGroups.openLatch())
                .addNext(new Delay(0.2))
//                .addNext(getIntakeParameters)
                .addNext(new VisualIntakeStage1(intakeParameters, robot.scoringMechanism.turret, robot.scoringMechanism.horizontalExtension))
                .addNext(commandGroups.setArmHeightVisionStack(intakeParameters))
                .addNext(new VisualIntakeStage2(intakeParameters, robot.scoringMechanism.turret, robot.scoringMechanism.horizontalExtension))
                .addNext(new Delay(0.1))
                .addNext(commandGroups.grabCone())
                .addNext(commandGroups.moveArm(Turret.ArmStates.TRANSFER_SAFE))
                .addNext(commandGroups.moveTurret(Turret.TurretStates.TRANSFER))
                .addNext(commandGroups.moveHorizontalExtension(HorizontalExtension.IN_POSITION))
                .addNext(commandGroups.moveArm(Turret.ArmStates.TRANSFER))
                .addNext(commandGroups.releaseCone())
                .addNext(commandGroups.closeLatch())
                .addNext(new Delay(0.1))
                .addNext(commandGroups.moveArm(Turret.ArmStates.TRANSFER_SAFE));
    }

    @Override
    public Team getTeam() {
        return Team.RED;
    }

    @Override
    public VisionMode getVisionMode() { return VisionMode.SPICE; }
}
