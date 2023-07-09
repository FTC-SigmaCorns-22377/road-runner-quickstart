package org.firstinspires.ftc.teamcode.Utils;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FeedbackController;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.MotionConstraint;

public class ProfiledPID implements FeedbackController {

	MotionConstraint verticalConstraint;
	MotionConstraint downConstraint;
	PIDCoefficients coefficients;
	BasicPIDFixed controller;
	ElapsedTime timer = new ElapsedTime();
	MotionProfile m_profile;
	double previousMotorTarget = 0;
	double m_targetPosition = 0;
	double m_state = 0;
	public ProfiledPID(MotionConstraint verticalConstraint, MotionConstraint downConstraint,
					   PIDCoefficients coefficients) {
		this.verticalConstraint = verticalConstraint;
		this.downConstraint = downConstraint;
		this.coefficients = coefficients;
		this.controller = new BasicPIDFixed(coefficients);
	}

	@Override
	public double calculate(double reference, double state) {
		generateMotionProfile(reference, state);
		m_targetPosition = getTargetPosition();
		m_state = state;
		double power = controller.calculate(m_targetPosition, m_state);
		if (power < 0) {
			power = Range.clip(power, -1, 1);
		}
		return power;
	}

	public double calculateNoMotionProfile(double reference, double state) {
		return controller.calculate(reference, state);
	}

	protected void generateMotionProfile(double reference, double state) {
		if (reference != previousMotorTarget || m_profile == null) {
			timer.reset();

			if (reference > previousMotorTarget) {
				m_profile = MotionProfileGenerator.generateSimpleMotionProfile(
						new MotionState(state, 0, 0),
						new MotionState(reference, 0, 0),
						verticalConstraint.max_velocity,
						verticalConstraint.max_acceleration
				);
			} else {
				m_profile = MotionProfileGenerator.generateSimpleMotionProfile(
						new MotionState(state, 0, 0),
						new MotionState(reference, 0, 0),
						downConstraint.max_velocity,
						downConstraint.max_acceleration
				);
			}

		}
		previousMotorTarget = reference;

	}

	public double getTargetPosition() {
		return m_profile.get(timer.seconds()).getX();
	}

	public double getVelocity() {
		return m_profile.get(timer.seconds()).getV();
	}

	public boolean isDone() {

		if (m_profile != null) {
			return (timer.seconds() > m_profile.duration()
					&& Math.abs(m_targetPosition - m_state) < 20)
					|| (timer.seconds() > m_profile.duration() + 2);
		}
		return Math.abs(m_targetPosition - m_state) < 20;
	}

	public void setPIDCoefficients(PIDCoefficients coefficients) {
		controller.setPIDCoefficients(coefficients);
	}
}
