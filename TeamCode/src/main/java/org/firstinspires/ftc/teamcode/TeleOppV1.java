package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.robot.subsytems.ArmV2;
import org.firstinspires.ftc.teamcode.robot.subsytems.Shoulder;

@TeleOp(name = "TeleOppV1", group = "Robot")
public class TeleOppV1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Shoulder shoulder = new Shoulder(
                hardwareMap.get(DcMotorEx.class,"shoulder"),
                hardwareMap.get(TouchSensor.class,"shoulderSensor2"));
        ArmV2 arm = new ArmV2(
                hardwareMap.get(DcMotorEx.class,"telescope"));
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        waitForStart();
        arm.setPosition(RobotConstants.arm_minPos);

        // Scan servo till stop pressed.
        while(opModeIsActive()){
            double left_y = gamepad1.left_stick_y;
            double right_y = gamepad1.right_stick_y;
            if (Math.abs(left_y)<0.1){
                left_y = 0;
            }
            if (Math.abs(right_y)<0.1){
                right_y = 0;
            }
            if (gamepad1.a) {
                shoulder.setPosition(RobotConstants.shoulder_dropOffPos);
            }
            if (gamepad1.b) {
                shoulder.setPosition(0);
            }

            if (gamepad1.x) {
                arm.setPosition(RobotConstants.arm_maxPos);
            }
            if (gamepad1.y) {
                arm.setPosition(RobotConstants.arm_minPos);
            }

            //double correction = shoulder.update();
            //arm.update();

            arm.moveArmManual(left_y);
            shoulder.moveShoulderManual(right_y);
            telemetry.addData("arm Position", arm.getEncoderValue() );
            telemetry.addData("arm Target",arm.getTarget());
            telemetry.addData("shoulder Position", shoulder.getEncoderValue() );
            telemetry.addData("shoulder Target", shoulder.getTarget());
            //telemetry.addData("shoulder correction", correction);
            telemetry.addData("stick", left_y);
            telemetry.update();
        }

    }
}
