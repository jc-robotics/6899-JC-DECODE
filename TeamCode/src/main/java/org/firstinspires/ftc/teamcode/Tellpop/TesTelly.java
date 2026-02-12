package org.firstinspires.ftc.teamcode.Tellpop;

import static org.firstinspires.ftc.teamcode.Libs.JCLibs.clamp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Core.DriveT;
import org.firstinspires.ftc.teamcode.Libs.PlayOpMode;

@TeleOp(name = "TesTelly", group = "Test")
public class TesTelly extends PlayOpMode{

    DriveT DTR = new DriveT();

    Gamepad PS4 = new Gamepad();

    @Override
    protected void preinitilize() {
        isTeleOp = true;

    }
    @Override
    protected void initialize() {


    }

    @Override
    protected void run(double dt) throws InterruptedException {

    }
}
