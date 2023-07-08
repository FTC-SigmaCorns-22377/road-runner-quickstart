package org.firstinspires.ftc.teamcode.OpModes;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.MotionConstraint;
import org.firstinspires.ftc.teamcode.Utils.ProfiledPID;

@Disabled
public class MotorFinder extends LinearOpMode {


	private static final double TAU = Math.PI * 2;
	private static final double MIN_RAW_SERVO_ANGLE = 0;
	private static final double MAX_RAW_SERVO_ANGLE = 1;
	double highPosition = 833;
	double setpoint = 150;
	PIDCoefficients coefficients = new PIDCoefficients(0.01, 0, 0);
	MotionConstraint upConstraint = new MotionConstraint(5000, 5000, 2000);
	MotionConstraint downConstraint = new MotionConstraint(5000, 5000, 2000);
	ServoImplEx turret;
	ProfiledPID pid = new ProfiledPID(upConstraint, downConstraint, coefficients);

	@Override
	public void runOpMode() throws InterruptedException {
		Servo aBrake = hardwareMap.get(Servo.class, "break");
		DcMotorEx rightHorizontal = hardwareMap.get(DcMotorEx.class, "rightHorizontal");
		rightHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
		DcMotorEx FrontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
		FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
		DcMotorEx FrontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
		FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		DcMotorEx leftHorizontal = hardwareMap.get(DcMotorEx.class, "leftHorizontal");
		leftHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		leftHorizontal.setDirection(DcMotorSimple.Direction.FORWARD);
		DcMotorEx vertical1 = hardwareMap.get(DcMotorEx.class, "vertical1");
		vertical1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		vertical1.setDirection(DcMotorSimple.Direction.FORWARD);
		DcMotorEx vertical2 = hardwareMap.get(DcMotorEx.class, "vertical2");
		vertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		vertical2.setDirection(DcMotorSimple.Direction.REVERSE);
		DcMotorEx BackLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
		BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		DcMotorEx BackRight = hardwareMap.get(DcMotorEx.class, "BackRight");
		BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		BackRight.setDirection(DcMotorSimple.Direction.FORWARD);
		Servo arm = hardwareMap.get(Servo.class, "arm");
		arm.setDirection(Servo.Direction.REVERSE);
		Servo claw = hardwareMap.get(Servo.class, "claw");
		Servo s2 = hardwareMap.get(Servo.class, "ch2");
		turret = hardwareMap.get(ServoImplEx.class, "turret");
		turret.setPwmRange(new PwmControl.PwmRange(500, 2500));
		Servo latch = hardwareMap.get(Servo.class, "latch");
		waitForStart();
		double turretAngle = Math.PI;

		double latchAngle = 0;
		while (opModeIsActive()) {
			turretAngle += -gamepad1.left_stick_y * 0.01;
			setpoint -= gamepad1.right_stick_y;
			setBasedTurretPositionSync(turretAngle);
			//s1.setPosition(turretAngle);
			// s0.setPosition(0.5);
//			arm.setPosition(0.3);
			claw.setPosition(0.5);

			double slidePosition = (leftHorizontal.getCurrentPosition() + rightHorizontal.getCurrentPosition()) / 2.0;
			double power = 0;

			leftHorizontal.setPower(power);
			rightHorizontal.setPower(power);
			telemetry.addData("slidePosition position", slidePosition);
			telemetry.addData("turretAngle angle", turretAngle);
			telemetry.addData("servo unit position", turret.getPosition());
			telemetry.update();
			if (gamepad1.triangle) {
				arm.setPosition(0.4);
			}
			if (gamepad1.square) {
				arm.setPosition(0.08);
			}
		}
	}

	private double radiansToServo(double radians) {
		return Range.scale(radians, 0, Math.PI * 2, MIN_RAW_SERVO_ANGLE, MAX_RAW_SERVO_ANGLE);
	}

	private double servoToRadians(double servoAngle) {
		return Range.scale(servoAngle, MIN_RAW_SERVO_ANGLE, MAX_RAW_SERVO_ANGLE, 0, TAU);
	}

	public void setBasedTurretPositionSync(double radians) {
		radians = Range.clip(radians, 0, TAU);
		if (radians == 0 || radians == TAU) { // turn to 0/360 degrees the fastest way
			double currentPosition = getBasedTurretPosition();
			if (currentPosition > Math.PI) { // if current position is closer to 360 than 0, turn to 360
				radians = TAU;
			} else if (currentPosition < Math.PI) { // if current position is closer to 0 than 360, turn to 0
				radians = 0;
			}
		}
		setRawTurretPositionSync(radiansToServo(radians));
	}

	public double getBasedTurretPosition() {
		return servoToRadians(getRawTurretPosition());
	}

	private void setRawTurretPositionSync(double position) {
		position = Range.clip(position, MIN_RAW_SERVO_ANGLE, MAX_RAW_SERVO_ANGLE);
		turret.setPosition(position);
	}

	private double getRawTurretPosition() { //TODO make private or protected
		return turret.getPosition();
	}
}
