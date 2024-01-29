package org.firstinspires.ftc.teamcode;


import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.text.DecimalFormat;

//@TeleOp(name="Robot: asdf", group="Robot")

public class asdf extends LinearOpMode{

    /* Declare OpMode members. */
    //public DcMotor  leftDrive   = null;
    //public DcMotor  rightDrive  = null;
    public DcMotor  slideLeft     = null;
    public DcMotor slideRight  = null;
    public DcMotor mtwo = null;

    public AHRS navx_device = null;
    public DcMotor mthree = null;
    // public Servo    leftClaw    = null;
    // public Servo    rightClaw   = null;

    double clawOffset = 0;
    public int count = 0;
    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    public int OUFBI = 0;

    @Override
    public void runOpMode() {
        this.navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"), AHRS.DeviceDataType.kProcessedData);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            boolean connected = navx_device.isConnected();
            telemetry.addData("1 navX-Device", connected ?
                    "Connected" : "Disconnected" );
            String gyrocal, magcal, yaw, pitch, roll, compass_heading;
            String fused_heading, ypr, cf, motion;
            DecimalFormat df = new DecimalFormat("#.##");

            if ( connected ) {
                gyrocal = (navx_device.isCalibrating() ?
                        "CALIBRATING" : "Calibration Complete");
                magcal = (navx_device.isMagnetometerCalibrated() ?
                        "Calibrated" : "UNCALIBRATED");
                yaw = df.format(navx_device.getYaw());
                pitch = df.format(navx_device.getPitch());
                roll = df.format(navx_device.getRoll());
                ypr = yaw + ", " + pitch + ", " + roll;
                compass_heading = df.format(navx_device.getCompassHeading());
                fused_heading = df.format(navx_device.getFusedHeading());
                if (!navx_device.isMagnetometerCalibrated()) {
                    compass_heading = "-------";
                }
                cf = compass_heading + ", " + fused_heading;
                if ( navx_device.isMagneticDisturbance()) {
                    cf += " (Mag. Disturbance)";
                }
                motion = (navx_device.isMoving() ? "Moving" : "Not Moving");
                if ( navx_device.isRotating() ) {
                    motion += ", Rotating";
                }
            } else {
                gyrocal =
                        magcal =
                                ypr =
                                        cf =
                                                motion = "-------";
            }
            telemetry.addData("2 GyroAccel", gyrocal );
            telemetry.addData("3 Y,P,R", ypr);
            telemetry.addData("4 Magnetometer", magcal );
            telemetry.addData("5 Compass,9Axis", cf );
            telemetry.addData("6 Motion", motion);
            telemetry.update();

        }
    }
}
