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
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.robot.subsytems.ArmV2;
import org.firstinspires.ftc.teamcode.robot.subsytems.DriveSystem;
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
        DriveSystem driveTrain = new DriveSystem(
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"));
        AHRS navx = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"), AHRS.DeviceDataType.kProcessedData);


        waitForStart();
        arm.setPosition(RobotConstants.arm_minPos);

        // Scan servo till stop pressed.
        while(opModeIsActive()){
            float left_y = gamepad1.left_stick_y;
            float right_y = gamepad1.right_stick_y;
            float left_x = gamepad1.left_stick_x;
            float right_x = gamepad1.right_stick_x;
            float yawCurr = navx.getYaw();
            if (Math.abs(left_y)<0.1){
                left_y = 0;
            }
            if (Math.abs(right_y)<0.1){
                right_y = 0;
            }
            if (Math.abs(left_x)<0.1){
                left_x = 0;
            }
            if (Math.abs(right_x)<0.1){
                right_x = 0;
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
            driveTrain.moveMethod(left_x, left_y, right_x, yawCurr);

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
