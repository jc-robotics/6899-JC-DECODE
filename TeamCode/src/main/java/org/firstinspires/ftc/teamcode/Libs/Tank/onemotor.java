package org.firstinspires.ftc.teamcode.Libs.Tank;

import org.firstinspires.ftc.teamcode.Libs.JCLibs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Libs.PlayOpMode;

@TeleOp(name = "onemotor", group = "onemotor")
public class onemotor extends PlayOpMode{
    HardwareMap map;
    @Override
    protected void initialize() {

        d.init(map);
        d.initializeMotor(d.motor1, DcMotor.Direction.FORWARD);
    } //Initializing
    //Clas Ref
    drivetrain d = new drivetrain();

    void spin(DcMotor motor, DcMotorSimple.Direction direction, double power){
        motor.setPower(power);

    }

    //Controller
    double lsx;
    void Controller(Gamepad gamepaD){
        lsx = -gamepaD.left_stick_x;

    }

    @Override
    protected void run(double dt) throws InterruptedException {
        Controller(gamepad1);
        spin(d.motor1, DcMotor.Direction.FORWARD, lsx);
        //Breaks for the Drive train
    }
}
