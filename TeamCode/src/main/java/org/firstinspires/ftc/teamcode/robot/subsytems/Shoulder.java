package org.firstinspires.ftc.teamcode.robot.subsytems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Shoulder {
    private DcMotorEx motor;
    private TouchSensor touch;

    public Shoulder(DcMotorEx motor, TouchSensor methT) {
        this.motor = motor;
        this.touch = methT;
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //up is encoder value negative
    public int getEncoderValue() {
        return this.motor.getCurrentPosition();
    }

    public boolean getTouchValue() {
        return this.touch.isPressed();
    }

    //input > 0 shoulder goes down
    public void moveShoulderManual(double input){
        if (this.touch.isPressed() && input > 0){
            this.motor.setPower(0);
            this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            this.motor.setPower(input);
        }
    }

}
