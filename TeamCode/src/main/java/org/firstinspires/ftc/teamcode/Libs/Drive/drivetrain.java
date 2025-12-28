package org.firstinspires.ftc.teamcode.Libs.Drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class drivetrain {
    public DcMotor FL,FR,BL,BR,im,FireL,FireR;
    public void init (HardwareMap map) {
        FL = map.get(DcMotor.class, "Front Left");
        FR = map.get(DcMotor.class, "Front Right");
        BL = map.get(DcMotor.class, "Back Left");
        BR = map.get(DcMotor.class, "Back Right");
        im = map.get(DcMotor.class, "intake");
        FireL = map.get(DcMotor.class, "FireL");
        FireR = map.get(DcMotor.class, "FireR");
    }

    void initializeMotor(DcMotor motor, DcMotor.Direction direction) {
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

    public void intake(DcMotor motor, DcMotor.Direction direction, double power) {
        if( power < 0){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setDirection(direction);
            motor.setPower(power);
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
}

