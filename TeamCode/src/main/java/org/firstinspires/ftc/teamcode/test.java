/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.robot.subsytems.DriveSystem;
import org.firstinspires.ftc.teamcode.robot.subsytems.LiftSystem;


/**
 * This particular OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: test", group="Robot")



public class test extends LinearOpMode {




    private LiftSystem lifter;
    private DriveSystem driveTrain;
    /* Declare OpMode members. */
    //public DcMotor  leftDrive   = null;
    //public DcMotor  rightDrive  = null;
    public DcMotorEx slideLeft     = null;
    public DcMotorEx slideRight  = null;
    public DcMotorEx mtwo = null;
    public DcMotorEx rightarm  = null;
    public DcMotorEx leftarm = null;
    float gamepadY;
    float gamepadX;

    public AHRS ahrs = null;
    public DcMotor mthree = null;
    public double top = 0;
    public Servo leftClaw    = null;
   // public Servo    rightClaw   = null;
    public static PIDFCoefficients TRANSLATIONAL_PID = new PIDFCoefficients(1, 0, 0, 0);

    double clawOffset = 0;
    public int count = 0;
    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.66;
    public static final double ARM_DOWN_POWER  = -0.65 ;

    @Override
    public void runOpMode() throws InterruptedException {
        /*
        this.lifter = new LiftSystem(
          new MotorEx(hardwareMap, "www", RobotConstants.Ticks_Per_Rev* RobotConstants.GR, RobotConstants.RPM/ RobotConstants.GR),
          new MotorEx(hardwareMap, "lll", RobotConstants.Ticks_Per_Rev* RobotConstants.GR, RobotConstants.RPM/ RobotConstants.GR),
          hardwareMap.get(TouchSensor.class, "resetLeft"),
          hardwareMap.get(TouchSensor.class, "resetRight")
        );

        this.driveTrain = new DriveSystem(
                new MotorEx(hardwareMap, "frontLeft", RobotConstants.Ticks_Per_Rev* RobotConstants.GR, RobotConstants.RPM/ RobotConstants.GR),
                new MotorEx(hardwareMap, "frontRight", RobotConstants.Ticks_Per_Rev* RobotConstants.GR, RobotConstants.RPM/ RobotConstants.GR),
                new MotorEx(hardwareMap, "backLeft", RobotConstants.Ticks_Per_Rev* RobotConstants.GR, RobotConstants.RPM/ RobotConstants.GR),
                new MotorEx(hardwareMap, "backRight", RobotConstants.Ticks_Per_Rev* RobotConstants.GR, RobotConstants.RPM/ RobotConstants.GR)
        );

        this.ahrs = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"), AHRS.DeviceDataType.kProcessedData);

        int soundID = hardwareMap.appContext.getResources().getIdentifier("tindeck_1", "raw", hardwareMap.appContext.getPackageName());
        int soundID2 = hardwareMap.appContext.getResources().getIdentifier("jjj", "raw", hardwareMap.appContext.getPackageName());

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            gamepadX = gamepad1.left_stick_x;
            gamepadY = gamepad1.left_stick_y;

            if (gamepad1.a) {
                this.lifter.prepReset();
            }
            if (gamepad1.b) {
                this.lifter.setPosition(34.3);
            }
            if(gamepad1.x) {
                this.lifter.setPosition(15);

            }

            if(gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                this.driveTrain.moveMethod(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, ahrs.getYaw());
            }

            this.lifter.update();
            telemetry.addData("target", this.lifter.getTarget());
            telemetry.addData("leftPos", this.lifter.getPositionRaw(0));
            telemetry.addData("rightPos", this.lifter.getPositionRaw(1));
            telemetry.addData("leftPower", this.lifter.getPower(0));
            telemetry.addData("rightPower", this.lifter.getPower(1));
            telemetry.addData("leftSwitch",this.lifter.getResetSwitch(0));
            telemetry.addData("rightSwitch",this.lifter.getResetSwitch(1));

            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.


         */
        }
    }
