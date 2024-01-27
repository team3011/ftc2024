package org.firstinspires.ftc.teamcode.robot.subsytems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Shoulder {
    private MotorEx ShoulderMotor;

    public Shoulder(MotorEx motor) {
        this.ShoulderMotor = motor;

        this.ShoulderMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public int getEncoderValue() {
        return this.ShoulderMotor.getCurrentPosition();
    }

}
