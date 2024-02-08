package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.robot.subsytems.ArmV2;
import org.firstinspires.ftc.teamcode.robot.subsytems.DriveSystem;
import org.firstinspires.ftc.teamcode.robot.subsytems.DriveSystemV2;
import org.firstinspires.ftc.teamcode.robot.subsytems.JulliansClaw;
import org.firstinspires.ftc.teamcode.robot.subsytems.Lift;
import org.firstinspires.ftc.teamcode.robot.subsytems.Shoulder;
import org.firstinspires.ftc.teamcode.robot.subsytems.Wrist;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name = "BarnesWantsFuckingLights", group = "Robot")

public class Lights extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {

       double spos = 0.2525;
        Servo lights = hardwareMap.get(Servo.class, "lights");

        waitForStart();

        while(opModeIsActive()){

           /* if (gamepad1.dpad_up) {
                 spos += 0.00495;
            }
            if (gamepad1.dpad_down) {
                spos -= 0.00495;
            } */
            if(gamepad1.a) { // heartbeat red
                spos = 0.42922;
            }
            if (gamepad1.b)  { // heartbeat blue
                spos = 0.438;
            }
            if(gamepad1.dpad_down) { // blue red cancer
                spos= 0.434165;
            }
            /*if (gamepad1.x) { // lava
                spos = .394565;
            }
            if (gamepad1.y) { //ocean
                spos = .384665;
            }
            if (gamepad1.dpad_up) { // blue flowy
                spos = .3218;
            } */
            if (gamepad1.dpad_right) { // red strobe
                spos = 0.4703;
            }
            if (gamepad1.dpad_left) { // blue strobe
                spos = 0.47525;
            }
            if (gamepad1.right_bumper) {
                spos += 0.00495;
            }
            if (gamepad1.left_bumper) {
                spos -= 0.00495;
            }
            lights.setPosition(spos);
            telemetry.addData("light value", spos);
            telemetry.update();
            sleep(50);
    }
}
}

