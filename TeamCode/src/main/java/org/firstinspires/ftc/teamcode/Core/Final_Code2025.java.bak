package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Libs.JCLibs.lerp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libs.Classes.Vector3;
import org.firstinspires.ftc.teamcode.Libs.PlayOpMode;

import java.util.Objects;

@TeleOp(name="Final Code", group="demo")
public class Final_Code2025 extends PlayOpMode {
    HardwareMap map;

    //Class Ref
    drivetrain d = new drivetrain();
    lift l = new lift();
    Sensor S = new Sensor();
    Final_Intake intake = new Final_Intake();

    //Controller
    Gamepad current;
    Gamepad previous;
    double lsx;
    double rsx;
    double lsy;
    boolean up;
    boolean down;
    boolean left;
    boolean right;
    double RT;
    double LT;
    boolean a;
    boolean b;

    //VARIABLES
    double threshold;
    double threshold_;
    double trs;
    double lsx_lerped = 0;
    double rsx_lerped = 0;
    double lsy_lerped = 0;
    double  Angle2;
    double rpower;
    int op;
    double Min;
    double Max;
    double Offset = 4390;
    int Value;
    int Dangle;

    boolean Speed;
    boolean Drive;
    double Speed1 = 0.6;
    double Speed2  =1;


    double zero = 0;

    public String Color;
    public int Green;
    public  int Blue;
    public int Red;
    public int Yellow;
    public int CurrentC;
    public ColorSensor Sensor;
    public Vector3 RGB;
    public boolean C;
    public int AVG;
    public String team;


    @Override
    protected void preinitilize()
    {
        isTeleOp = true;
    }


    @Override
    protected void initialize() {
        telemetry.addData("Status: ","Initializing Hardware....");

        map = hardwareMap;

        l.init(map);
        d.init(map);
        intake.init(map);
        S.init(map);
        //Drive Train Motor Setup
        d.initializeMotor(d.LB, DcMotor.Direction.REVERSE);
        d.initializeMotor(d.LF, DcMotor.Direction.FORWARD);
        d.initializeMotor(d.RF, DcMotor.Direction.FORWARD);
        d.initializeMotor(d.RB, DcMotor.Direction.REVERSE);
        //Arm Motor Setup
        l.initializeEncoderMotor(l.DC1);
        l.initializeEncoderMotor(l.DC2);
        //Intake Setup
        intake.Initialize(zero);
         current = new Gamepad();


         previous = new Gamepad();



        l.m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l.m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Min = l.DC1.getCurrentPosition();
        Max = Min + Offset;


        Color="";

        telemetry.addData("Status: ","Initialization Complete");
        telemetry.update();
    }
    // so switch case dont take over run
    void SwitchCase(){
        switch (Drive ? 1:0) {
            case 0:
                switch (Speed ? 1 : 0) {
                    case 0://false
                        d.move(lsy_lerped * Speed1, lsx_lerped * Speed1, rsx_lerped * Speed1);
                        break;

                    case 1://true
                        d.move(lsy_lerped * Speed2, lsx_lerped * Speed2, rsx_lerped * Speed2);
                        break;
                }
                break;
            case 1:
                d.move(lsy_lerped * 0, lsx_lerped * 0, rsx_lerped * 0);
                telemetry.speak("Click L3 Genius ");
                break;
        }
    }

    //Place any telemetry data in function
    void Telemetry(){
        telemetry.addData("CP1", l.DC1.getCurrentPosition());
        telemetry.addData("CP2",l.DC2.getCurrentPosition());
        telemetry.addData("Block", S.Color);
        telemetry.addData("Color", S.CurrentC);
        telemetry.addData("Red", S.Red);
        telemetry.addData("Blue", S.Blue);
        telemetry.addData("Green", S.Green);
        telemetry.addData("Yellow: ",S.Yellow);
        telemetry.addData("Current Value",Value);
        telemetry.addData("Avg C: ",S.AVG);
        telemetry.addData("Team: ",S.Team);
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("Left Wheel Position: ", intake.lwheel.getPosition());
        telemetry.addData("Right Wheel Position: ", intake.rwheel.getPosition());
        telemetry.addData("Wrist Yaw Position: ", intake.wyaw.getPower());
        telemetry.addData("Arm Yaw Position: ", intake.ayaw.getPower());
        telemetry.addData("Arm Pitch Power: ",intake.apitch.getPower());
        telemetry.update();


    }

    //Assigns controller inputs to variable
    void Controller(Gamepad gamepaD){

        //Thump sticks
        lsy = gamepaD.left_stick_y;
        rsx = -gamepaD.right_stick_x;
        lsx = -gamepaD.left_stick_x;

        //Dpad
        up = gamepaD.dpad_up;
        down = gamepaD.dpad_down;
        left = gamepaD.dpad_left;
        right = gamepaD.dpad_right;

        //Face buttons
        a = gamepaD.a;
        b = gamepaD.b;
        RT = gamepaD.right_trigger;
        LT = gamepaD.left_trigger;
        previous.copy(current);


        // Store the gamepad values from this loop iteration in
        // currentGamepad1/2 to be used for the entirety of this loop iteration.
        // This prevents the gamepad values from changing between being
        // used and stored in previousGamepad1/2.
        current.copy(gamepaD);

    }

    public void Switch1(){
        C=!C;

    }

    public void Switch2(){
        C= true;

    }

    public void apitchdown(double LeftTrigger){
        intake.apitch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.apitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.apitch.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.apitch.setPower(LeftTrigger*1);


    }

    public void apitchup(double RightTrigger){
        intake.apitch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.apitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.apitch.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.apitch.setPower(RightTrigger*1);


    }


    @Override

    protected void run(double dt) throws InterruptedException {
        Angle2 = (l.DC2.getCurrentPosition()/2786.2)*360;
        Dangle = (Value/360)*2786;
        Controller(gamepad1);
        threshold = 0.8;
        threshold_ = 1;
        trs = 1;
        op = 1000;

        rpower = LT - RT;
        l.m1.setPower(rpower);

        lsx_lerped = lerp(lsx_lerped, lsx, 0.9);
        rsx_lerped = lerp(rsx_lerped, rsx, 0.9);
        lsy_lerped = lerp(lsy_lerped, lsy, 0.9);
        SwitchCase();


        S.Sense();
        Telemetry();
        if(a)
        {
            Value += 50;
        }
        if(b)
        {
            Value -= 50
        ;
        }
        //Vertical
        if (up) {
            l.Arm(5180,trs,2350,1);
        }
        //Down
        if (down) {

            l.Arm(2200,trs,3200,threshold);
        }
        //Reset to start position
        if(left) {
            l.Arm(0,trs,0,threshold);
        }
        // Rung 1 Part 1
        if (right) {

            l.Arm(3400,trs,2350,threshold);
        }

        if (current.left_stick_button && !previous.left_stick_button) {
           Speed =! Speed;
        }
        if (current.right_stick_button && !previous.right_stick_button) {
            Drive =! Drive;
        }

        // TeleOP controls

        intake.wyaw.setPower(gamepad2.left_stick_x*0.5);
        intake.wyaw.setDirection(CRServo.Direction.REVERSE);

        intake.ayaw.setPower(gamepad2.right_stick_x);

        intake.wpitch.setPower(gamepad2.left_stick_y);


        apitchup(gamepad2.right_trigger);
        apitchdown(gamepad2.left_trigger);


        if(gamepad2.dpad_down){
            intake.apitch.setPower(0);
        }


        if (gamepad2.cross) {

            intake.WheelPos(0,0,Servo.Direction.REVERSE,Servo.Direction.FORWARD);

        }

        if (gamepad2.square) {

            intake.WheelPos(.5,.5,Servo.Direction.FORWARD,Servo.Direction.REVERSE);


        }



        // Sensor Code


        //Switching teams
        if (gamepad2.left_bumper) {
            Switch1();
        }

        if(gamepad2.right_bumper){
            Switch2();
        }


        if(Objects.equals(S.Color, "Wrong Color") && Objects.equals(S.Team, "Red")){

            intake.WheelPos(.5,.5,Servo.Direction.FORWARD,Servo.Direction.REVERSE);

        }

        if(Objects.equals(S.Color, "Wrong Color") && Objects.equals(S.Team, "Blue")){

            intake.WheelPos(.5,.5,Servo.Direction.FORWARD,Servo.Direction.REVERSE);

        }




    }
}