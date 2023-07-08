package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled

public class ClawAngleTester extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		double outAngle = 0.63;
		double inAngle = 0.03;

		Servo claw = hardwareMap.get(Servo.class, "intake");
		waitForStart();
		double angle = outAngle;
		while (opModeIsActive()) {

			if (gamepad1.triangle) {
				angle = outAngle;
			}
			if (gamepad1.cross) {
				angle = inAngle;
			}

			claw.setPosition(angle);
			telemetry.addData("claw angle: ", angle);
			telemetry.update();
		}
	}
}
