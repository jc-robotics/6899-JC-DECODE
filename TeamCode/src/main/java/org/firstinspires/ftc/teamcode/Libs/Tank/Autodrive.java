package org.firstinspires.ftc.teamcode.Libs.Tank;

import org.firstinspires.ftc.teamcode.Libs.JCLibs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Libs.PlayOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoDrive" , group = "Autonomous")
public class Autodrive extends PlayOpMode{
    drivetrain d = new drivetrain();
    HardwareMap map;
    JCLibs J;



    @Override
    protected void initialize() {
        map = hardwareMap;

        d.init(map);
        d.initializeMotor(d.BL, DcMotor.Direction.REVERSE);
        d.initializeMotor(d.FL, DcMotor.Direction.REVERSE);
        d.initializeMotor(d.FR, DcMotor.Direction.FORWARD);
        d.initializeMotor(d.BR, DcMotor.Direction.FORWARD);

        d.initializeMotor(d.im, DcMotor.Direction.FORWARD);
        d.initializeMotor(d.screw, DcMotorSimple.Direction.FORWARD);
        d.initializeMotor(d.FireL, DcMotor.Direction.FORWARD);
        d.initializeMotor(d.FireR, DcMotorSimple.Direction.REVERSE);
    }

    @Override
    protected void run(double dt) throws InterruptedException {

    }
}
