package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Neutrino.Rev2MSensor.Rev2mDistanceSensorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.MotionConstraint;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Utils.ProfiledPID;

@Config
public class VerticalExtension extends Subsystem {

	public static double HIGH_POSITION = 26;
	public static double HIGH_POSITION_teleop = 26.3;
	public static double HIGH_POSITION_RIGHT_AUTO = HIGH_POSITION_teleop + 0.05;

	public static double HIGH_POSITION_LEFT_AUTO = HIGH_POSITION_teleop + 0.2;

	public static double MID_POSITION = 15.5;

	public static double MID_POSITION_teleop = 16;

	public final static double IN_POSITION = 0;
	static final double PULLEY_CIRCUMFERENCE = 4.409;
	static final double counts_per_revolution = 145.090909;
	public static double Kp = 0.25;
	public static double Kd = 1.8 * Math.sqrt(Kp * 0.0015);
	public static double max_accel = 250;
	public static double max_velocity = 300;

	double power = 0;

	public static double DISTANCE_FOR_CONE = 8.5; // 9 inches or less means we still have the cone

	public Rev2mDistanceSensorEx distanceSensor;

	protected double slideTargetPosition = 0;
	protected double Kg = 0.09499;
	MainScoringMechanism.MechanismStates state = MainScoringMechanism.MechanismStates.BEGIN;
	DcMotorEx vertical1;
	DcMotorEx vertical2;
	PIDCoefficients coefficients = new PIDCoefficients(Kp, 0, Kd);
	PIDCoefficients emergency_kp = new PIDCoefficients(Kp * 10, 0, Kd);

	MotionConstraint upConstraint = new MotionConstraint(max_accel/ 1.5, max_accel / 2, max_velocity / 1.5);
	MotionConstraint downConstraint = new MotionConstraint(max_accel, max_accel, max_velocity);
	ProfiledPID controller = new ProfiledPID(upConstraint, downConstraint, coefficients);
	private VoltageSensor batteryVoltageSensor;
	protected double current = 0;
	protected boolean is_auto = true;

	public void commonInit(HardwareMap hwMap) {
		vertical1 = hwMap.get(DcMotorEx.class, "vertical1");
		vertical2 = hwMap.get(DcMotorEx.class, "vertical2");
		this.distanceSensor = hwMap.get(Rev2mDistanceSensorEx.class,"cone");
		this.distanceSensor.setRangingProfile(Rev2mDistanceSensorEx.RANGING_PROFILE.HIGH_SPEED);
		// TODO, set direction
		vertical2.setDirection(DcMotorSimple.Direction.REVERSE);
		vertical1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		vertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		this.batteryVoltageSensor = hwMap.voltageSensor.iterator().next();
	}

	@Override
	public void initAuto(HardwareMap hwMap) {
		commonInit(hwMap);
		vertical1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		vertical2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		vertical1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		vertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	@Override
	public void initTeleop(HardwareMap hwMap) {
		commonInit(hwMap);
		upConstraint = new MotionConstraint(max_accel, max_accel / 2, max_velocity);
		downConstraint = new MotionConstraint(max_accel, max_accel / 2, max_velocity);

		vertical1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		vertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		is_auto = false;
	}

	@Override
	public void periodic() {

		if (is_auto && getSlidePosition() < 5 && Math.signum(power) == -1 && Math.abs(countsToInches(getVelocity())) < 2 && !slideIsDown()) {
			controller.setPIDCoefficients(emergency_kp);
			System.out.println("EMERGENCY KP MODE POG");
		} else {
			controller.setPIDCoefficients(coefficients);
		}

		updatePID();
		Dashboard.packet.put("distance to cone / deposit", getDistanceToDeposit());

	}

	@Override
	public void shutdown() {
		vertical1.setPower(0);
		vertical2.setPower(0);

	}

	protected void updatePID() {
		double measuredPosition = getSlidePosition();
		power = controller.calculate(slideTargetPosition, measuredPosition);
		if (slideTargetPosition > IN_POSITION * 2) {
			power += Kg;
		} else {
			power -= Kg;
		}

		if (currentLimitExceeded()) {
			power /= 4;
		}

		vertical1.setPower(power);
		vertical2.setPower(power);
		Dashboard.packet.put("measured slide position", measuredPosition);
		Dashboard.packet.put("target slide position", controller.getTargetPosition());
		Dashboard.packet.put("slide power", power);
		current = vertical1.getCurrent(AMPS);
		Dashboard.packet.put("vertical current amps",current);
	}

	/**
	 * get position of the linear slides
	 *
	 * @return average encoder position of the slides
	 */
	public double getSlidePosition() {
		return countsToInches((vertical1.getCurrentPosition() + vertical2.getCurrentPosition()) / 2.0);
	}

	public double getSlideTargetPosition() {
		return slideTargetPosition;
	}

	public void updateTargetPosition(double targetpos) {
		this.slideTargetPosition = targetpos;
	}

	public boolean isMovementFinished() {
		return controller.isDone();
	}

	public double countsToInches(double counts) {
		return (counts / counts_per_revolution) * PULLEY_CIRCUMFERENCE;
	}

	public boolean currentLimitExceeded() {
		return Math.abs(current) > 5 && !is_auto;
	}

	public double getCurrent() {
		return current;
	}

	public boolean coneIsStillInDeposit() {
		return slideIsDown() && getDistanceToDeposit() < DISTANCE_FOR_CONE;
	}

	public double getDistanceToDeposit() {
		return distanceSensor.getDistance(DistanceUnit.INCH);
	}

	public boolean slideIsDown() {
		return getSlidePosition() < 0.65;
	}

	public double getVelocity() {
		return vertical1.getVelocity();
	}

}
