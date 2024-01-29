package org.firstinspires.ftc.teamcode.robot.subsytems;


import java.lang.Math;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class LiftSystem {
    /*
    public boolean resetting = false;
    private ElapsedTime resettingTimer;
    public MotorEx leftSlider = null;
    public MotorEx rightSlider = null;
    public TouchSensor leftSense = null;
    public TouchSensor rightSense = null;
    public PIDCoefficients coefficients;
    private int target = 0;
    private int lastTarget = 0;
    private ElapsedTime timer = new ElapsedTime();
    private PIDFController controller;
    private MotionProfile profile;
    private double leftPower = 0;
    private double rightPower = 0;

    public LiftSystem(MotorEx left, MotorEx right, TouchSensor leftT, TouchSensor rightT) {
        this.leftSlider = left;
        this.rightSlider = right;
        this.leftSense = leftT;
        this.rightSense = rightT;

        this.leftSlider.setRunMode(MotorEx.RunMode.RawPower);
        this.rightSlider.setRunMode(MotorEx.RunMode.RawPower);
        this.leftSlider.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        this.rightSlider.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        this.rightSlider.setInverted(true);

        this.coefficients = new PIDCoefficients(RobotConstants.kP, RobotConstants.kI, RobotConstants.kD);
        this.controller = new PIDFController(this.coefficients, 0, 0, 0, (x,y)-> RobotConstants.kG);

        this.profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0,0,0),
                new MotionState(0,0,0),
                RobotConstants.maxVel,
                RobotConstants.maxAccel,
                RobotConstants.maxJerk
        );

    }

    public double getPos(MotorEx motor) {
        return motor.getCurrentPosition() * RobotConstants.Ticks_Per_Rev * RobotConstants.GR;
    }

    public int getTarget() {
        return this.target;
    }

    public void setPosition (double targetPos) {
        this.target = (int) (RobotConstants.TPR_CM * RobotConstants.GR * targetPos);
        if (this.target != this.lastTarget) {
            this.profile = MotionProfileGenerator.generateSimpleMotionProfile(
                    new MotionState(this.leftSlider.getCurrentPosition(),0,0),
                    new MotionState(target,0,0),
                    RobotConstants.maxVel,
                    RobotConstants.maxAccel,
                    RobotConstants.maxJerk
            );
            this.lastTarget = this.target;
            this.timer.reset();
        }
    }

    public int getPositionRaw (int i) {
        if (i == 0) {
            return this.leftSlider.getCurrentPosition();
        }
        else {
            return this.rightSlider.getCurrentPosition();
        }
    }
    public void setPositionRaw(int j) {
        this.target = j;
        if (this.target != this.lastTarget) {
            this.profile = MotionProfileGenerator.generateSimpleMotionProfile(
                    new MotionState(this.leftSlider.getCurrentPosition(), 0, 0),
                    new MotionState(this.target, 0, 0),
                    RobotConstants.maxVel,
                    RobotConstants.maxAccel,
                    RobotConstants.maxJerk
            );
            this.lastTarget = this.target;
            this.timer.reset();
        }
    }

    public void prepReset() {
        this.resetting = true;
        this.resettingTimer = new ElapsedTime();
    }

    public boolean getResetSwitch(int j) {
        if (j == 0) {
            return this.leftSense.isPressed();
        } else {
            return this.rightSense.isPressed();
        }
    }

    public void resetSliders() {
        if ((this.resettingTimer.seconds() < 2) && !(this.leftSense.isPressed() && this.rightSense.isPressed())) {
            if (this.rightSense.isPressed()) {
                this.rightSlider.set(0);
                this.rightSlider.resetEncoder();
            } else {
                this.rightSlider.set(-.2);
            }
            if (this.leftSense.isPressed()) {
                this.leftSlider.set(0);
                this.leftSlider.resetEncoder();
            } else {
                this.leftSlider.set(-.2);
            }
        }
        this.resetting = false;
        this.setPositionRaw(0);
    }

    public double getPower(int b) {
        if (b == 0) {
            return this.leftPower;
        }
        else {
            return this.rightPower;
        }
    }

    public void update() {
        if (this.resetting) {
            this.resetSliders();
        }
        else {
            MotionState state = this.profile.get(this.timer.seconds());
            this.controller.setTargetPosition(state.getX());
            this.controller.setTargetVelocity(state.getV());
            this.controller.setTargetAcceleration(state.getA());
            int leftPosition = this.leftSlider.getCurrentPosition();
            int rightPosition = this.rightSlider.getCurrentPosition();
            double errorLeft = controller.update(leftPosition);
            double errorRight = controller.update(rightPosition);

            if (Math.abs(leftPosition - rightPosition) > RobotConstants.maxDifference) {
                this.leftSlider.set(0);
                this.rightSlider.set(0);
            }
            else {
                if ((errorLeft < 0) && (this.leftSense.isPressed())) {
                    this.leftSlider.set(0);
                    this.leftSlider.resetEncoder();
                } else if ((errorLeft > 0) && (leftPosition > RobotConstants.max_Ticks_Raw * RobotConstants.GR)) {
                    this.leftSlider.set(0);
                } else if ((errorLeft < 0) && (leftPosition > rightPosition)) {
                    this.leftSlider.set(errorLeft * RobotConstants.Reduction);
                } else if ((errorLeft > 0) && (leftPosition > rightPosition + 100)) {
                    this.leftSlider.set(errorLeft * RobotConstants.Reduction);
                } else {
                    this.leftSlider.set(errorLeft);
                }

                if ((errorRight < 0) && (this.rightSense.isPressed())) {
                    this.rightSlider.set(0);
                    this.rightSlider.resetEncoder();
                } else if ((errorRight > 0) && (rightPosition > RobotConstants.max_Ticks_Raw * RobotConstants.GR)) {
                    this.rightSlider.set(0);
                } else if ((errorRight < 0) && (leftPosition < rightPosition)) {
                    this.rightSlider.set(errorRight * RobotConstants.Reduction);
                } else if ((errorRight > 0) && (leftPosition < rightPosition - 100)) {
                    this.rightSlider.set(errorRight * RobotConstants.Reduction);
                } else {
                    this.rightSlider.set(errorRight);
                }
            }

        }

    }

     */
}
