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

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.robot.subsytems.Shoulder;
import org.firstinspires.ftc.teamcode.robot.subsytems.Arm;
import org.firstinspires.ftc.teamcode.robot.subsytems.DriveSystem;
import org.firstinspires.ftc.teamcode.robot.subsytems.Wrist;

import java.text.DecimalFormat;

/*
 * This OpMode scans a single servo back and forward until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a Robot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
//@TeleOp(name = "Robot: hellonearth", group = "Robot")
//@Config
public class hellonearth extends LinearOpMode {

    public static double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    public static double MULTIPLIER = 0.25;
    public static double ANGLE;
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    private Wrist main;
    DriveSystem driveTrain;
    MotorEx motor;

    Shoulder shoulder;

    // Define class members
    Wrist wrist;
    Servo   servo;
    Servo servo2;
    double  position;
    double  wposition;
    double  aposition;
    boolean rampUp = true;
    AHRS navX;
    Arm ourArm;
    String yaw;
    Shoulder newShould;


    @Override
    public void runOpMode() {

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        //servo = hardwareMap.get(Servo.class, "servo");
        //servo2 = hardwareMap.get(Servo.class, "servo2");
        //this.wrist = new wrist(servo, servo2);
        //MotorEx motor = new MotorEx(hardwareMap, "telescope", RobotConstants.Ticks_Per_Rev* RobotConstants.GR, RobotConstants.RPM/ RobotConstants.GR);
        //MotorEx fuck = new MotorEx(hardwareMap, "shoulder", RobotConstants.Ticks_Per_Rev* RobotConstants.GR, RobotConstants.RPM/ RobotConstants.GR);
        MotorEx frontLeft = new MotorEx(hardwareMap, "frontLeft", 28*19.2, 312/19.2);
        MotorEx frontRight = new MotorEx(hardwareMap, "frontRight", 28*19.2, 312/19.2);
        MotorEx backLeft = new MotorEx(hardwareMap, "backLeft", 28*19.2, 312/19.2);
        MotorEx backRight = new MotorEx(hardwareMap, "backRight", 28 * 19.2, 312 / 19.2);
        Servo nerd = hardwareMap.get(Servo.class, "servo");
        Servo left = hardwareMap.get(Servo.class, "left");
        Servo right = hardwareMap.get(Servo.class, "right");
        //Shoulder shoulder = new Shoulder(fuck,TouchSensor touch);
        this.driveTrain = new DriveSystem(frontLeft, frontRight, backLeft, backRight);
        this.navX = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"), AHRS.DeviceDataType.kProcessedData);
        this.main = new Wrist(left, right);
        this.main.moveWrist(.5);
        //fuck.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //this.ourArm = new Arm(motor);
        DecimalFormat df = new DecimalFormat("#.##");
        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){

            yaw = df.format(this.navX.getYaw());

            if (gamepad1.right_bumper) {
                position += INCREMENT;
            }
            if (gamepad1.left_bumper) {
                position -= INCREMENT;
            }
            if (gamepad1.a) {
                position = 0.76;
            }
            if (gamepad1.b) {
                position = 0.1;
            }
            if (gamepad1.dpad_up) {
                wposition += INCREMENT;
            }
            if (gamepad1.dpad_down) {
                wposition -= INCREMENT;
            }
            if (gamepad1.y) {
                wposition = .05;
            }
            if (gamepad1.x) {
          //      this.ourArm.setPosition(5);

            }

            // Display the current value
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.addData("Position", nerd.getPosition() );
            telemetry.addData("Position Wrist", wposition );
            telemetry.update();

            // Set the servo to the new position and pause;
            this.driveTrain.moveMethod(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, this.navX.getYaw());
            this.main.moveWrist(wposition);
            //this.ourArm.update();
            //nerd.setPosition(position);
       //     motor.set(gamepad1.left_stick_y*MULTIPLIER);
            //fuck.set(gamepad1.right_stick_y*MULTIPLIER);
            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.addData("Shoulder Angle", (this.motor.getCurrentPosition()/1680*360)-ANGLE);
        telemetry.update();
    }
}
