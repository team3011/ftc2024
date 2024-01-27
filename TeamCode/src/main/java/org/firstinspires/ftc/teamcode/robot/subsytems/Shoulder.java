package org.firstinspires.ftc.teamcode.robot.subsytems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Shoulder {
    private MotorEx ShoulderMotor;
    private TouchSensor touch;

    public Shoulder(MotorEx motor, TouchSensor methT) {
        this.ShoulderMotor = motor;
        this.touch = methT;
    //github support lyander-jon dumo
        this.ShoulderMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public int getEncoderValue() {
        return this.ShoulderMotor.getCurrentPosition();
    }

    public boolean getTouchValue() {
        return this.touch.isPressed();
    }

}
