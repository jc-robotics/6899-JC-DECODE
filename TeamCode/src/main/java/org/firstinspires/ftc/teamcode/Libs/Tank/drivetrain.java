package org.firstinspires.ftc.teamcode.Libs.Tank;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

//
public class drivetrain {
    public DcMotor FL,FR,BL,BR,im,screw,FireL,FireR; // Declaration of Motors
    public void init (HardwareMap map) {
        FL = map.get(DcMotor.class, "FrontLeft");
        FR = map.get(DcMotor.class, "FrontRight");
        BL = map.get(DcMotor.class, "BackLeft");
        BR = map.get(DcMotor.class, "BackRight");
        im = map.get(DcMotor
                .class, "intake");
        screw = map.get(DcMotor.class, "screw");
        FireL = map.get(DcMotor.class, "FireL");
        FireR = map.get(DcMotor.class, "FireR");
    } // function Hardware Map Motors so we can tell each motor what to do

    void initializeMotor(DcMotor motor, DcMotor.Direction direction) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
    } // Function to initialize Motor

    public void move(double x, double y, double turn) {
        FL.setPower(y+x+turn);
        BL.setPower(y-x+turn);
        FR.setPower(y-x-turn);
        BR.setPower(y+x-turn);

    } // Function to make Drive train move

    public void intake(boolean button,DcMotor motor, DcMotor.Direction direction, double power) {
        if(button){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setDirection(direction);
            motor.setPower(power);
        } else {
                 motor.setPower(0);

        } //Intake function

    }

    public void intake1(boolean button1,boolean button2, DcMotor motor, DcMotor.Direction direction, double power) {
        if(button1){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setDirection(direction);
            motor.setPower(power);
        } else if(button2){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setDirection(direction);
            motor.setPower(-power);

        } else {
            motor.setPower(0);
        }//Intake function

    }

    public void Fire(DcMotor motor, DcMotor.Direction direction, float power) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
        motor.setPower(power);
    } //Shooting Function
    
}

