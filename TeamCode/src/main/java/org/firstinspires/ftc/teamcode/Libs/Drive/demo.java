package org.firstinspires.ftc.teamcode.Libs.Drive;

import static org.firstinspires.ftc.teamcode.Libs.JCLibs.lerp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Libs.PlayOpMode;

@TeleOp(name = "Drive", group = "demo")
public class demo extends PlayOpMode {
    HardwareMap map;

    //Clas Ref
    drivetrain d = new drivetrain();

    //Controller
    double lsx;
    double rsx;
    double lsy;

    //Variables
    double theta;
    double Speed1 = 0.6;
    double Speed2 = 1;
    double lsx_lerped = 0;
    double rsx_lerped = 0;
    double lsy_lerped = 0;
    boolean a;
    boolean b;
    boolean y;
    boolean x;
    boolean Drive;
    boolean Speed;
    float RT;

    @Override
    protected void initialize() {
        map = hardwareMap;

        d.init(map);
        d.initializeMotor(d.BL, DcMotor.Direction.REVERSE);
        d.initializeMotor(d.FL, DcMotor.Direction.REVERSE);
        d.initializeMotor(d.FR, DcMotor.Direction.FORWARD);
        d.initializeMotor(d.BR, DcMotor.Direction.FORWARD);

        d.initializeMotor(d.im, DcMotor.Direction.FORWARD);
        d.initializeMotor(d.im2, DcMotorSimple.Direction.FORWARD);
        d.initializeMotor(d.FireL, DcMotor.Direction.FORWARD);
        d.initializeMotor(d.FireR, DcMotorSimple.Direction.REVERSE);
    } //Initializing Motors

    void SwitchCase(){
        switch (Drive ? 1:0) {
            case 0:
                switch (Speed ? 1 : 0) {
                    case 0://false
                        d.move(lsx_lerped * Speed1, lsy_lerped * Speed1, rsx_lerped * Speed1);
                        break;

                    case 1://true
                        d.move(lsx_lerped * Speed2, lsy_lerped * Speed2, rsx_lerped * Speed2);
                        break;
                }
                break;
            case 1:
                d.move(lsx_lerped * 0, lsy_lerped * 0, rsx_lerped * 0);
                telemetry.speak("Click L3 Genius ");
                break;
        }
    } //Movement of Drive train use joysticks on controller

    void Controller(Gamepad gamepaD){
        lsy = gamepaD.left_stick_y;
        lsx = -gamepaD.left_stick_x;
        rsx = -gamepaD.right_stick_x;
        a = gamepaD.a;
        b = gamepaD.b;
        x = gamepaD.x;
        y = gamepaD.y;
        RT = gamepaD.right_trigger;
    } //Assigning Gamepad Controls

    void intake(){
        d.intake(a, d.im, DcMotor.Direction.REVERSE, Speed1);
        d.intake1(x, y, d.im2, DcMotorSimple.Direction.FORWARD, Speed1);
    }

    void Shooter(){
        d.Fire(d.FireL,DcMotor.Direction.FORWARD,RT);
        d.Fire(d.FireR, DcMotorSimple.Direction.REVERSE,RT);
    } // Function to operate shooting Artifacts

    @Override
    protected void run(double dt) throws InterruptedException {
        Controller(gamepad1);
        //Breaks for the Drive train
        lsx_lerped = lerp(lsx_lerped, lsx, 0.9);
        rsx_lerped = lerp(rsx_lerped, rsx, 0.9);
        lsy_lerped = lerp(lsy_lerped, lsy, 0.9);
        intake();
        SwitchCase();
        Shooter();
    }
}
