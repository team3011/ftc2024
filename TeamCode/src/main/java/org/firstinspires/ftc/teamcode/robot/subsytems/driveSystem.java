package org.firstinspires.ftc.teamcode.robot.subsytems;



import java.lang.Math;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotConstants;
public class driveSystem {

    public float xValue = 0;
    public float yValue = 0;
    public float yawValue = 0;
    MotorEx frontLeft;
    MotorEx frontRight;
    MotorEx backLeft;
    MotorEx backRight;
    AHRS navX;


    public driveSystem(MotorEx fL, MotorEx fR, MotorEx bL, MotorEx bR) {
        this.frontLeft = fL;
        this.frontRight = fR;
        this.backLeft = bL;
        this.backRight = bR;

        this.frontLeft.setRunMode(Motor.RunMode.RawPower);
        this.frontRight.setRunMode(Motor.RunMode.RawPower);
        this.backLeft.setRunMode(Motor.RunMode.RawPower);
        this.backRight.setRunMode(Motor.RunMode.RawPower);
    }


    public void straighten() {

    }


    public void moveMethod(float stickX, float stickY, float rightX, float yawCurr) {

        double max;

        double axial   = stickX;
        double lateral = (-1) * stickY;
        double yaw     =  rightX;

        double pi = 3.1415926;

        double gyro_degrees = yawCurr;
        double gyro_radians = gyro_degrees * pi/180;
        double temp = lateral * Math.cos(gyro_radians) + axial * Math.sin(gyro_radians);
        axial = (-1) * lateral * Math.sin(gyro_radians) + lateral * Math.cos(gyro_radians);
        lateral = temp;


        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normsalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }
        if (this.yawValue > robotConstants.yawMax) {
            if (this.yawValue > 5) {
                leftFrontPower = yaw/-10;
                leftBackPower = yaw/-10;
                rightBackPower = yaw/10;
                rightFrontPower = yaw/10;
            }
            else if (this.yawValue < 5) {
                leftFrontPower = yaw/10;
                leftBackPower = yaw/10;
                rightBackPower = yaw/-10;
                rightFrontPower = yaw/-10;
            }
        }
        this.frontLeft.set(leftFrontPower);
        // frontLeft = y - x
        this.frontRight.set(rightFrontPower);
        // frontRight = y + x
        this.backLeft.set(leftBackPower);
        // backleft = -y - x
        this.backRight.set(rightBackPower);
        // backright = -y + x
    }




}
