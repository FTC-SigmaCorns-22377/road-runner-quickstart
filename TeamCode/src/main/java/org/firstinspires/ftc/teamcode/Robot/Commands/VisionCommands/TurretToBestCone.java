package org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;


public class TurretToBestCone extends Command {
    @Override
    public void init() {

    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean completed() {
        return false;
    }

    @Override
    public void shutdown() {

    }
//    private Turret turret;
//    private Vision vision;
//    private boolean targetLock;
//    private boolean foundTarget = false;
//    private Cone target = null;
//    private boolean allowPerfect;
//    private boolean allowFar;
//    private boolean allowClose;
//    private boolean allowDeadzone;
//    private boolean isCompleted = false;
//
//
//    public TurretToBestCone(Turret turret, Vision vision, boolean targetLock, boolean allowPerfect, boolean allowFar, boolean allowClose, boolean allowDeadzone) {
//        super(turret, vision);
//        this.turret = turret;
//        this.vision = vision;
//        this.targetLock = targetLock;
//        this.allowPerfect = allowPerfect;
//        this.allowFar = allowFar;
//        this.allowClose = allowClose;
//        this.allowDeadzone = allowDeadzone;
//    }
//
//    @Override
//    public void init() {
//        this.findTarget();
//    }
//
//    @Override
//    public void periodic() {
//        if (!this.foundTarget || !this.targetLock) this.findTarget();
//        if (Math.abs(this.turret.getTurretPosition() - this.target.servoAngle) <= .02) this.isCompleted = true;
//    }
//
//    @Override
//    public boolean completed() {
//       return this.isCompleted;
//    }
//
//    @Override
//    public void shutdown() {
//        turret.shutdown();
//    }
//
//    private void findTarget() {
//        assert this.vision.backCam.pipe instanceof Optimized;
//        if (((Optimized) this.vision.backCam.pipe).perfect != null && this.allowPerfect) this.target = ((Optimized) this.vision.backCam.pipe).perfect;
//        if (((Optimized) this.vision.backCam.pipe).far != null && this.allowFar) this.target = ((Optimized) this.vision.backCam.pipe).far;
//        if (((Optimized) this.vision.backCam.pipe).close != null && this.allowClose) this.target = ((Optimized) this.vision.backCam.pipe).close;
//        if (((Optimized) this.vision.backCam.pipe).deadzone != null && this.allowDeadzone) this.target = ((Optimized) this.vision.backCam.pipe).deadzone;
//        if (this.target != null) {
//            this.foundTarget = true;
//            this.turret.setTurretPositionSync(this.target.servoAngle);
//            Dashboard.packet.put("SERVO", this.target.servoAngle);
//        }
//    }
//
}
