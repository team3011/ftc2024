package org.firstinspires.ftc.teamcode.robot.subsytems;



import java.lang.Math;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotConstants;
public class DriveSystem {

    public float xValue = 0;
    public float yValue = 0;
    public float yawValue = 0;
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;
    AHRS navX;


    public DriveSystem(DcMotorEx fL, DcMotorEx fR, DcMotorEx bL, DcMotorEx bR) {
        this.frontLeft = fL;
        this.frontRight = fR;
        this.backLeft = bL;
        this.backRight = bR;

       /* this.frontLeft.(Motor.RunMode.RawPower);
        this.frontRight.setRunMode(Motor.RunMode.RawPower);
        this.backLeft.setRunMode(Motor.RunMode.RawPower);
        this.backRight.setRunMode(Motor.RunMode.RawPower);*/
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
        if (this.yawValue > RobotConstants.yawMax) {
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
        this.frontLeft.setPower(leftFrontPower);
        // frontLeft = y - x
        this.frontRight.setPower(rightFrontPower);
        // frontRight = y + x
        this.backLeft.setPower(leftBackPower);
        // backleft = -y - x
        this.backRight.setPower(rightBackPower);
        // backright = -y + x
    }




}
