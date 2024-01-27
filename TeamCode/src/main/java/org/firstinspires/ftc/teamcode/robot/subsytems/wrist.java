package org.firstinspires.ftc.teamcode.robot.subsytems;

import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotConstants;

public class wrist {

    public Servo leftWrist;
    public Servo rightWrist;
    private double target;
    private double target2;


    public wrist(Servo leftJoint, Servo rightJoint) {
        this.leftWrist = leftJoint;
        this.rightWrist = rightJoint;

        this.rightWrist.setDirection(Servo.Direction.REVERSE);
    }

    public void setTarget(double input) {
        this.target = input;
    }
    public void moveWrist(double target) {

        // check for current encoder value of shoulder
        // convert above value to degree accounting for starting pos
        // move wrist to pos based on 90 - above degreee, will have to find out triangle details for the wrist

        this.leftWrist.setPosition(target);
        this.rightWrist.setPosition(target);
    }

    public double getTarget() {
        return this.target;
    }
}
