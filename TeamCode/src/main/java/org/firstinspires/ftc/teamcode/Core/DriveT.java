package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveT {
    DcMotor LF, RF, LB, RB;

    public void init (HardwareMap map){
        LF = map.get(DcMotor.class, "Fin");
        RF = map.get(DcMotor.class, "Sasaki");
        LB = map.get(DcMotor.class, "Sato");
        RB = map.get(DcMotor.class, "Rakib");
    }
    void initializeMotor(DcMotor motor, DcMotorSimple.Direction direction) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
    }
    public void move(double x, double y, double turn) {
        LF.setPower(y+x+turn);
        RF.setPower(y-x-turn);
        LB.setPower(y-x+turn);
        RB.setPower(y+x-turn);
    }
}
