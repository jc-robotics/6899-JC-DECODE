package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

public class drivetrain {
    DcMotor LF, LB, RF, RB;
    public void init (HardwareMap map){
        LF = map.get(DcMotor.class, "FrontLeft");
        RF = map.get(DcMotor.class, "FrontRight");
        LB = map.get(DcMotor.class, "BackLeft");
        RB = map.get(DcMotor.class, "BackRight");
    }
    //Motor Setup
    void initializeMotor(DcMotor motor,  DcMotor.Direction direction) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);

    }
    void initializeEncoderMotor(DcMotor motor,  DcMotor.Direction direction) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);

    }
    //Moves the Bot Duh
    public void move(double x, double y, double turn) {
        LF.setPower(y+x+turn);
        LB.setPower(y-x-turn);
        RF.setPower(y-x+turn);
        RB.setPower(y+x-turn);
    }


}



