package org.firstinspires.ftc.teamcode.robot.subsytems;


import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Arm {
    /*
    public boolean resetting = false;
    private ElapsedTime resettingTimer;
    public MotorEx motor;
    public TouchSensor rightSense = null;
    public PIDCoefficients coefficients;
    private int target = 0;
    private int lastTarget = 0;
    private ElapsedTime timer = new ElapsedTime();
    private PIDFController controller;
    private MotionProfile profile;
    private double leftPower = 0;
    private double rightPower = 0;

    public Arm(MotorEx motor) {
        this.motor = motor;

        this.motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

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
                    new MotionState(this.motor.getCurrentPosition(),0,0),
                    new MotionState(target,0,0),
                    RobotConstants.maxVel,
                    RobotConstants.maxAccel,
                    RobotConstants.maxJerk
            );
            this.lastTarget = this.target;
            this.timer.reset();
        }
    }


    public void setPositionRaw(int j) {
        this.target = j;
        if (this.target != this.lastTarget) {
            this.profile = MotionProfileGenerator.generateSimpleMotionProfile(
                    new MotionState(this.motor.getCurrentPosition(), 0, 0),
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



    public double getPower(int b) {
        if (b == 0) {
            return this.leftPower;
        }
        else {
            return this.rightPower;
        }
    }

    public void update() {

            MotionState state = this.profile.get(this.timer.seconds());
            this.controller.setTargetPosition(state.getX());
            this.controller.setTargetVelocity(state.getV());
            this.controller.setTargetAcceleration(state.getA());
            int leftPosition = this.motor.getCurrentPosition();
            double errorArm = controller.update(leftPosition);

               this.motor.set(errorArm);
            }


     */
        }

