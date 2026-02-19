package org.firstinspires.ftc.teamcode.Libs.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class drivetrain {
    public DcMotor FL,FR,BL,BR,im,FireL,FireR,screw;
    public void init (HardwareMap map) {
        FL = map.get(DcMotor.class, "FrontLeft");
        FR = map.get(DcMotor.class, "FrontRight");
        BL = map.get(DcMotor.class, "BackLeft");
        BR = map.get(DcMotor.class, "BackRight");
        im = map.get(DcMotor.class, "intake");
        FireL = map.get(DcMotor.class, "FireL");
        FireR = map.get(DcMotor.class, "FireR");
        screw = map.get(DcMotor.class, "screw");
    }

    public void initializeMotor(DcMotor motor, DcMotor.Direction direction) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
    }

    public void move(double x, double y, double turn) {
        FL.setPower(y+x+turn);
        BL.setPower(y-x+turn);
        FR.setPower(y-x-turn);
        BR.setPower(y+x-turn);

    }

    public void intake(boolean button1, boolean button2, DcMotor motor, DcMotor.Direction direction, double power) {
        if( button1){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setDirection(direction);
            motor.setPower(power);
        } else if (button2){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setDirection(direction);
            motor.setPower(-power);
        } else {
            motor.setPower(0);
        }

    }

    public void Fire(DcMotor motor, DcMotor.Direction direction, double power) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
        motor.setPower(power);
    }

    public void lina(DcMotor motor, DcMotor.Direction direction){
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
        motor.setPower(.5);
    }


}
