package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.robot.subsytems.Shoulder;

@TeleOp(name = "TeleOppV1", group = "Robot")
@Config
public class TeleOppV1  extends LinearOpMode {
    Shoulder shoulder;

    @Override
    public void runOpMode() {
        Shoulder shoulder = new Shoulder(
                hardwareMap.get(DcMotorEx.class,"shoulder"),
                hardwareMap.get(TouchSensor.class,"shoulderSensor2"));

        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){
            double left_y = gamepad1.left_stick_y;
            if (Math.abs(left_y)<0.1){
                left_y = 0;
            }
            if (gamepad1.a) {
                shoulder.moveToEncoder(-2000);
            }

            //shoulder.moveShoulderManual(left_y);
            telemetry.addData("Position", shoulder.getEncoderValue() );
            telemetry.addData("Sensor", shoulder.getTouchValue());
            telemetry.addData("stick", left_y);
            telemetry.update();
        }

    }
}
