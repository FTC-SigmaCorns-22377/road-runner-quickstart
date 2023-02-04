package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.RR_quickstart.util.BasedMath.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.RR_quickstart.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RR_quickstart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RR_quickstart.util.BasedMath;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.Break.ToggleBreak;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RoadrunnerHoldPose;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;
import org.firstinspires.ftc.teamcode.visionPipelines.SleeveDetection;

import java.util.HashMap;
import java.util.Objects;

@Autonomous
public class VanillaLatteAuto extends BaseAuto {
    Pose2d startPose = new Pose2d(-36, 66.5,Math.toRadians(-90));

    @Override
    public void setRobotPosition() {
        robot.drivetrain.setPose(startPose);
    }

    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        TrajectoryVelocityConstraint slowVelocity = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL_slow,DriveConstants.MAX_ANG_VEL_slow, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint slowAcceleration = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL_slow);

        ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism,robot.drivetrain);

        Vector2d goToPole = new Vector2d(-36, 23);
        Pose2d rotateFaceMedium = shiftRobotRelative(new Pose2d(-34, 27,Math.toRadians(21.8816732757)),-3,-2);

        Pose2d parkLeft = new Pose2d(-4, 16, Math.toRadians(180));
        Pose2d parkCenter = new Pose2d(-35, 12.5, Math.toRadians(0));
        Pose2d parkRight = new Pose2d(-58, 12.5, Math.toRadians(0));

        HashMap<SleeveDetection.ParkingPosition, Pose2d> parking = new HashMap<>();

        parking.put(SleeveDetection.ParkingPosition.LEFT, parkLeft);
        parking.put(SleeveDetection.ParkingPosition.CENTER, parkCenter);
        parking.put(SleeveDetection.ParkingPosition.RIGHT, parkRight);

        Trajectory scoring1 = robot.drivetrain.getBuilder().trajectoryBuilder(startPose,false)
                .splineToConstantHeading(goToPole,Math.toRadians(270),slowVelocity,slowAcceleration)
                .build();
        Trajectory scoring2 = robot.drivetrain.getBuilder().trajectoryBuilder(scoring1.end(),false)
                .lineToLinearHeading(rotateFaceMedium,slowVelocity,slowAcceleration)
                .build();
        Trajectory parkTraj = robot.drivetrain.getBuilder().trajectoryBuilder(scoring2.end(),false)
                .lineToLinearHeading(Objects.requireNonNull(parking.get(parkingPosition)))
                .build();

        Command auto = followRR(scoring1).addNext(followRR(scoring2));
        auto.addNext(new RoadrunnerHoldPose(robot,scoring2.end()));
        auto.addNext(new ToggleBreak(robot.drivetrain));
        for (int i = 0; i < 5; ++i) {
            addCycle(auto,commandGroups);
        }
        auto.addNext(commandGroups.moveVerticalExtension(VerticalExtension.MID_POSITION))
                .addNext(commandGroups.depositCone());
        auto.addNext(new ToggleBreak(robot.drivetrain));

        auto = auto.addNext(followRR(parkTraj));
        return auto;
    }

    public void addCycle(Command command, ScoringCommandGroups commandGroups) {
        command.addNext(multiCommand(commandGroups.moveVerticalExtension(VerticalExtension.MID_POSITION),
                        commandGroups.moveToIntakingLeftAuto(),
                        commandGroups.moveHorizontalExtension(HorizontalExtension.PRE_EMPTIVE_EXTEND)))
                .addNext(commandGroups.moveHorizontalExtension(HorizontalExtension.mostlyAutoExtension_MID))
                .addNext(commandGroups.collectConeAuto(HorizontalExtension.autoExtension_MID));
    }
}
