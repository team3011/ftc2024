package org.firstinspires.ftc.teamcode.robot.subsytems;



import java.lang.Math;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class DriveSystem {

    public float xValue = 0;
    public float yValue = 0;
    public float yawValue = 0;
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;
    boolean checker = false;
    Encoder perpEncoder;
    Encoder parEncoder;
    AHRS navX;


    public DriveSystem(DcMotorEx fL, DcMotorEx fR, DcMotorEx bL, DcMotorEx bR, Encoder parallel, Encoder perpindicular) {
        this.frontLeft = fL;
        this.frontRight = fR;
        this.backLeft = bL;
        this.backRight = bR;
        this.parEncoder = parallel;
        this.perpEncoder = perpindicular;

        //this.parEncoder.setDirection(Encoder.Direction.REVERSE);


        this.frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        this.backLeft.setDirection(DcMotorEx.Direction.REVERSE);

       this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }


    public void resetEncoder() {
        this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public int perpReturn() {
        return this.perpEncoder.getCurrentPosition();
    }

    public int parReturn() {
        return this.parEncoder.getCurrentPosition();
    }

    public double perpReturnInches() {
        return TwoWheelTrackingLocalizer.encoderTicksToInches(perpReturn());
    }

    public double parReturnInches() {
        return TwoWheelTrackingLocalizer.encoderTicksToInches(parReturn());
    }




    public void moveMethod(float x, float y, float rx, float yawCurr) {


        double botHeading = (yawCurr) +90;
        double pi = 3.1415926;
        botHeading *= pi/180;

        if (yawCurr >= RobotConstants.yawMax) {
            checker = true;
        }
        if (checker) {
            if (yawCurr <= RobotConstants.yawCheck) {
                checker = false;
            }
            else {
                x = 0;
                y = 0;
                rx = (yawCurr - RobotConstants.yawCheck);
            }

        }

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing



        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);




        /*  double max;

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
        */
    }




}
