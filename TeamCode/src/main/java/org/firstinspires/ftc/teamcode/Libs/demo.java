package org.firstinspires.ftc.teamcode.Libs;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.Libs.pedroPathing.drivetrain;

@TeleOp(name = "demo", group = "demo")
    public class demo extends OpMode {
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
        boolean Drive;
        boolean Speed;
        float RT;



        @Override
        public void init() {
            map = hardwareMap;

            d.init(map);
            d.initializeMotor(d.BL, DcMotor.Direction.REVERSE);
            d.initializeMotor(d.FL, DcMotor.Direction.FORWARD);
            d.initializeMotor(d.FR, DcMotor.Direction.REVERSE);
            d.initializeMotor(d.BR, DcMotor.Direction.FORWARD);

            d.initializeMotor(d.im, DcMotor.Direction.FORWARD);
            d.initializeMotor(d.FireL, DcMotor.Direction.FORWARD);
            d.initializeMotor(d.FireR, DcMotorSimple.Direction.REVERSE);
            d.initializeMotor(d.screw, DcMotor.Direction.FORWARD);
        }

        void SwitchCase(){
            switch (Drive ? 1:0) {
                case 0:
                    switch (Speed ? 1 : 0) {
                        case 0://false
                            d.move(lsx_lerped * Speed1, lsy_lerped * Speed1, rsx_lerped * Speed1);
                            d.intake(d.im, DcMotor.Direction.REVERSE, lsy * Speed1);
                            break;

                        case 1://true
                            d.move(lsx_lerped * Speed2, lsy_lerped * Speed2, rsx_lerped * Speed2);
                            d.intake(d.im, DcMotor.Direction.REVERSE  , lsy * Speed2);
                            break;
                    }
                    break;
                case 1:
                    d.move(lsx_lerped * 0, lsy_lerped * 0, rsx_lerped * 0);
                    d.intake(d.im, DcMotor.Direction.FORWARD, lsy * 0);
                    telemetry.speak("Click L3 Genius ");
                    break;
            }
        }

        void Controller(Gamepad gamepaD){
            lsy = gamepaD.left_stick_y;
            lsx = -gamepaD.left_stick_x;
            rsx = -gamepaD.right_stick_x;
            a = gamepaD.a;
            b = gamepaD.b;
            y = gamepaD.y;
            RT = gamepaD.right_trigger;
        }

        void Shooter(){
            if(gamepad1.xWasPressed()) {
                d.Fire(d.FireL, DcMotor.Direction.REVERSE, RT *.34);
                d.Fire(d.FireR, DcMotorSimple.Direction.FORWARD, RT * .34);
            } else if (gamepad1.yWasPressed()) {
                d.Fire(d.FireL, DcMotor.Direction.FORWARD, RT *.43);
                d.Fire(d.FireR, DcMotorSimple.Direction.REVERSE, RT * .43);
            }
        }

        void screw(){
            if(b) {
                d.lina(d.screw, DcMotor.Direction.FORWARD);
            }else if (y) {
                d.lina(d.screw, DcMotor.Direction.REVERSE);
            }else {
                d.screw.setPower(0);
            }

        }

        @Override
        public void loop() {
            Controller(gamepad1);
//            lsx_lerped = lerp(lsx_lerped, lsx, 0.9);
//            rsx_lerped = lerp(rsx_lerped, rsx, 0.9);
//            lsy_lerped = lerp(lsy_lerped, lsy, 0.9);
            SwitchCase();
            screw();

            Shooter();

            telemetry.addData("f1: ", d.FireR.getPower());
            telemetry.addData("f1: ", d.FireL.getPower());
            telemetry.update();
        }
}

