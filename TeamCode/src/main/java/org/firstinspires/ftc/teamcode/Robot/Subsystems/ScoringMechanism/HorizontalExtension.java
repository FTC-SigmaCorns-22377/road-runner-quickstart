package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.MotionConstraint;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Utils.ProfiledPID;

public class HorizontalExtension extends Subsystem {


	MainScoringMechanism.MechanismStates state = MainScoringMechanism.MechanismStates.BEGIN;

	DcMotorEx leftMotor;
	DcMotorEx rightMotor;

	PIDCoefficients coefficients = new PIDCoefficients(0.013,0,0);
	MotionConstraint upConstraint = new MotionConstraint(4000,5000,1800);
	MotionConstraint downConstraint = new MotionConstraint(4000,5000,1800);

	ProfiledPID controller = new ProfiledPID(upConstraint,downConstraint,coefficients);


	public final static double IN_POSITION = 6;
	public final static double SAFE_POSITION = 300;
	public final static double CLOSE_INTAKE = 100;
	public final static double EXTENSION1 = 500;
	public final static double PRE_EMPTIVE_EXTEND = 100;
	public final static double TELE_CYCLE_EXTENSION = 359; // todo figure out our max safe extension
	public final static double autoExtension = 395;
	public final static double mostlyAutoExtension = autoExtension - 30;
	public final static double autoExtension_MID = 400;
	public final static double mostlyAutoExtension_MID = autoExtension_MID - 30;
	protected double targetPosition = IN_POSITION;

	private static final double TICKS_TO_INCH = .03;
	private static final double INCH_TO_TICKS = 33.3333333333;

	public void commonInit(HardwareMap hwMap) {
		leftMotor = hwMap.get(DcMotorEx.class, "leftHorizontal");
		rightMotor = hwMap.get(DcMotorEx.class, "rightHorizontal");
		// TODO, set direction
		rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
	}
	@Override
	public void initAuto(HardwareMap hwMap) {
		commonInit(hwMap);
		leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}
	@Override
	public void initTeleop(HardwareMap hwMap) {
		commonInit(hwMap);
		leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	@Override
	public void periodic() {

		updatePID();
		Dashboard.packet.put("horizontal ticks", getSlidePosition());
		Dashboard.packet.put("horizontal inches", getSlidePositionInches());
	}

	@Override
	public void shutdown() {

	}


	protected void updatePID() {
		double measuredPosition = getSlidePosition();
		double power = controller.calculate(targetPosition,measuredPosition);
		Dashboard.packet.put("measured horizontal position",measuredPosition);
		Dashboard.packet.put("target horizontal position",controller.getTargetPosition());

		leftMotor.setPower(power);
		rightMotor.setPower(power);
	}

	public void setTargetPosition(double targetPosition) {
		this.targetPosition = targetPosition;
	}

	/**
	 * get position of the linear slides
	 * @return average encoder position of the slides
	 */
	public double getSlidePosition() {
		int leftPosition = leftMotor.getCurrentPosition();
		int rightPosition = rightMotor.getCurrentPosition();
		return (leftPosition + rightPosition) / 2.0;
	}

	public double getSlideTargetPosition() {
		return targetPosition;
	}
	public boolean isMovementFinished() {
		return controller.isDone();
	}

	private double ticksToInches(double ticks) {
		return (ticks - IN_POSITION) * TICKS_TO_INCH;
	}
	private double inchesToTicks(double inches) {
		return (inches - ticksToInches(IN_POSITION)) * INCH_TO_TICKS;
	}

	public double getSlidePositionInches() {
		return ticksToInches(getSlidePosition());
	}
	public void setTargetPositionInches(double targetPositionInches) {
		this.targetPosition = inchesToTicks(targetPositionInches);
	}
}
