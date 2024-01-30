package org.firstinspires.ftc.teamcode.robot.subsytems;



import java.lang.Math;
import java.util.Locale;

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
    private double headingToMaintain = 0;
    private String whatHeadingDo;


    public DriveSystem(DcMotorEx fL, DcMotorEx fR, DcMotorEx bL, DcMotorEx bR, AHRS n, Encoder parallel, Encoder perpindicular) {
        this.frontLeft = fL;
        this.frontRight = fR;
        this.backLeft = bL;
        this.backRight = bR;
        this.parEncoder = parallel;
        this.perpEncoder = perpindicular;
        this.navX = n;

        //this.parEncoder.setDirection(Encoder.Direction.REVERSE);


        this.frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        this.backLeft.setDirection(DcMotorEx.Direction.REVERSE);

       this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       //this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       //this.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void testMotors(int delay, double power) throws InterruptedException {
        this.frontLeft.setPower(power);
        Thread.sleep(delay);
        this.frontLeft.setPower(0);
        this.frontRight.setPower(power);
        Thread.sleep(delay);
        this.frontRight.setPower(0);
        this.backLeft.setPower(power);
        Thread.sleep(delay);
        this.backLeft.setPower(0);
        this.backRight.setPower(power);
        Thread.sleep(delay);
        this.backRight.setPower(0);
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


    public double getYaw(){
        return this.navX.getYaw();
    }
    /** main thread for this class, commands the motors to do required movements
     *
     * @param leftStickX commands robot strafing movements
     * @param leftStickY commands robot forward and backward movements
     * @param rightStickX commands robot's rotation
     */
    public void drive(double leftStickX, double leftStickY, double rightStickX){
        double y = leftStickY * RobotConstants.MULTIPLIER;
        double x = -leftStickX * 1.1 * RobotConstants.MULTIPLIER; // Counteract imperfect strafing
        double rx = rightStickX * RobotConstants.MULTIPLIER; //what way we want to rotate



        //imu is reversed somehow and for some reason, any instance should refer to it as neg
        //double robotHeading = -this.imu.getAngularOrientation().firstAngle;
        double robotHeading = this.navX.getYaw();

        /*
        if(rx == 0){ //this means that we're trying to maintain our current heading

            //prevents the motors from working when they realistically cant
            boolean isWithinAngularTolerance =
                    Math.abs(this.figureOutWhatIsShorter()) < RobotConstants.ANGULAR_TOLERANCE;

            //we turn if we're not within a tolerance
            if(!isWithinAngularTolerance){
                rx = this.figureOutWhatIsShorter(); //retrieves direction, magnitude overwritten
                rx = this.setToMinimumTurningSpeed(rx); //overwrites magnitude to minimum speed
            }

            this.whatHeadingDo =
                    String.format(Locale.getDefault(),"Maintaining Desired Heading of %.2f",
                            Math.toDegrees(headingToMaintain));
        }else{
            this.whatHeadingDo = "Turning";
            //we're going to maintain our new heading once we've stopped turning.
            //not before we've turned
            this.headingToMaintain = robotHeading;
        }

         */


        //triangle """magic"""
        //double rotX = x * Math.cos(robotHeading) - y * Math.sin(robotHeading);
        //double rotY = x * Math.sin(robotHeading) + y * Math.cos(robotHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        //double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        //double frontLeftPower = (rotY + rotX - rx)  / denominator;
        //double frontRightPower = (rotY - rotX + rx) / denominator;
        //double backLeftPower = (rotY - rotX - rx)   / denominator;
        //double backRightPower = (rotY + rotX + rx)  / denominator;

        //this.frontLeft.setPower(frontLeftPower);
        //this.frontRight.setPower(frontRightPower);
        //this.backLeft.setPower(backLeftPower);
        //this.backRight.setPower(backRightPower);

        this.frontLeft.setPower(y);
        this.frontRight.setPower(y);
        this.backLeft.setPower(y);
        this.backRight.setPower(y);
    }

    /** removes interference from sticks by setting as zero if below a tolerance
     *
     * @param stickInput stick
     * @return Zero if input was below tolerance, spits back input if above
     */
    /*private double setZeroIfBelowTolerance(double stickInput){
    ''    if(Math.abs(stickInput) < RobotConstants.STICK_TOLERANCE){
            return 0;
        }
        return stickInput;
    }

     */

    /** Determines what direction would be shorter to turn in when trying to maintain our current
     *  heading.
     * @return the shorter difference in heading
     */
    private double figureOutWhatIsShorter() {
        double result;
        double reading = this.navX.getYaw();
        double oppositeButEqualReading;

        if (reading > 0) {
            oppositeButEqualReading = reading - 2 * Math.PI;
        } else {
            oppositeButEqualReading = reading + 2 * Math.PI;
        }

        double normalReadingDifference = Math.abs(this.headingToMaintain - reading);
        double oppositeReadingDifference = Math.abs(this.headingToMaintain - oppositeButEqualReading);

        boolean isOppositeReadingShorter =
                normalReadingDifference > oppositeReadingDifference;

        if (isOppositeReadingShorter) {
            result = this.headingToMaintain - oppositeButEqualReading;
        } else {
            result = this.headingToMaintain - reading;
        }
        return result;
    }

    /** changes our current turning speed to a turning speed that allows us to rotate
     *
     * @param rx our current turning speed
     * @return modified turning speed
     */
    private double setToMinimumTurningSpeed(double rx){

        if(Math.abs(rx) < RobotConstants.MINIMUM_TURNING_SPEED) {
            if (rx < 0) {
                return -RobotConstants.MINIMUM_TURNING_SPEED;
            } else {
                return RobotConstants.MINIMUM_TURNING_SPEED;
            }
        }else{
            return rx;
        }
    }

    public String getWhatHeadingDo(){
        return this.whatHeadingDo;
    }

    //public double getCurrentHeadingDegrees(){
    //    return Math.toDegrees(-this.imu.getAngularOrientation().firstAngle);
    //}



}
