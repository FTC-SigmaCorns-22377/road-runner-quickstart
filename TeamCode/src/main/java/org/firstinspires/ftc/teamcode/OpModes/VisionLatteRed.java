package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.RR_quickstart.util.BasedMath.shiftRobotRelative;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.RR_quickstart.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RR_quickstart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.Brake.ToggleBrake;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RoadrunnerHoldPose;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands.GetIntakeParameters;
import org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands.VisualIntakeStage1;
import org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands.VisualIntakeStage2;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;
import org.firstinspires.ftc.teamcode.Utils.Team;
import org.firstinspires.ftc.teamcode.VisionUtils.IntakeParameters;
import org.firstinspires.ftc.teamcode.VisionUtils.VisionMode;
import org.firstinspires.ftc.teamcode.visionPipelines.SleeveDetection;

import java.util.HashMap;
import java.util.Objects;

@Disabled
public class VisionLatteRed extends BaseAuto {
    Pose2d startPose = new Pose2d(-36, 66.5, Math.toRadians(-90));

    @Override
    public void setRobotPosition() {
        robot.drivetrain.setPose(startPose);
    }


    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        TrajectoryVelocityConstraint slowVelocity = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL_slow, DriveConstants.MAX_ANG_VEL_slow, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint slowAcceleration = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL_slow);

        ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain, robot.backCamera);
        Vector2d goToPole = new Vector2d(-36, 23);
//        Vector2d goToPole = new Vector2d(-36, 23);
        Pose2d rotateFaceMedium = shiftRobotRelative(new Pose2d(-35, 26, Math.toRadians(21.8816732757)), -1.1, -2);
        Pose2d parkLeft = new Pose2d(-3, 16, Math.toRadians(90));
        Pose2d parkCenter = new Pose2d(-35, 14.5, Math.toRadians(90));
        Pose2d parkRight = new Pose2d(-60, 12.5, Math.toRadians(0));

        HashMap<SleeveDetection.ParkingPosition, Pose2d> parking = new HashMap<>();

        parking.put(SleeveDetection.ParkingPosition.LEFT, parkLeft);
        parking.put(SleeveDetection.ParkingPosition.CENTER, parkCenter);
        parking.put(SleeveDetection.ParkingPosition.RIGHT, parkRight);

        Trajectory scoring1 = robot.drivetrain.getBuilder().trajectoryBuilder(startPose, false)
                .splineToConstantHeading(goToPole, Math.toRadians(270), slowVelocity, slowAcceleration)
                .build();
        Trajectory scoring2 = robot.drivetrain.getBuilder().trajectoryBuilder(scoring1.end(), false)
                .lineToLinearHeading(rotateFaceMedium, slowVelocity, slowAcceleration)
                .build();

        Trajectory parkTraj = robot.drivetrain.getBuilder().trajectoryBuilder(scoring2.end(), false)
                .lineToLinearHeading(Objects.requireNonNull(parking.get(parkingPosition)))
                .build();

        Command auto = followRR(scoring1).addNext(followRR(scoring2));
        auto.addNext(new RoadrunnerHoldPose(robot, scoring2.end()));
        auto.addNext(new ToggleBrake(robot.drivetrain));
        for (int i = 0; i < 5; ++i) {
            addCycle(auto, commandGroups);
        }
        auto.addNext(commandGroups.moveVerticalExtension(VerticalExtension.MID_POSITION + .5))
                .addNext(new Delay(0.1))
                .addNext(commandGroups.depositCone());
        auto.addNext(new ToggleBrake(robot.drivetrain));

        auto = auto.addNext(followRR(parkTraj));
        return auto;
    }

    public void addCycle(Command command, ScoringCommandGroups commandGroups) {
        GetIntakeParameters getIntakeParameters = new GetIntakeParameters(robot.scoringMechanism.turret, robot.backCamera, robot.scoringMechanism.horizontalExtension);
        IntakeParameters intakeParameters = getIntakeParameters.getIntakeParameters();
        command.addNext(commandGroups.moveVerticalExtension(VerticalExtension.MID_POSITION + .5))
                .addNext(commandGroups.depositConeAsync())
                .addNext(commandGroups.openLatch())
                .addNext(new Delay(0.2))
                .addNext(getIntakeParameters)
                .addNext(new VisualIntakeStage1(intakeParameters, robot.scoringMechanism.turret, robot.scoringMechanism.horizontalExtension))
                .addNext(commandGroups.setArmHeightVisionStack(intakeParameters))
                .addNext(new VisualIntakeStage2(intakeParameters, robot.scoringMechanism.turret, robot.scoringMechanism.horizontalExtension))
                .addNext(new Delay(0.1))
                .addNext(commandGroups.grabCone())
                .addNext(commandGroups.moveArm(Turret.ArmStates.TRANSFER_SAFE))
                .addNext(commandGroups.moveTurret(Turret.TurretStates.TRANSFER))
                .addNext(commandGroups.moveHorizontalExtension(HorizontalExtension.IN_POSITION))
                .addNext(commandGroups.moveArm(Turret.ArmStates.TRANSFER))
                .addNext(new Delay(0.1))
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
    public VisionMode getVisionMode() { return VisionMode.LATTE; }
}

