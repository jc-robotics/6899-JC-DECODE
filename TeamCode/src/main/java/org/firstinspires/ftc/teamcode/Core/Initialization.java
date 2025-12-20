package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Libs.JCLibs.clamp;
import static org.firstinspires.ftc.teamcode.Libs.JCLibs.lerp;
import static org.firstinspires.ftc.teamcode.Libs.JCLibs.round;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libs.Classes.Vector3;
import org.firstinspires.ftc.teamcode.Libs.PlayOpMode;


import java.util.Objects;

public class Initialization extends PlayOpMode {
    HardwareMap map;

    telemetry.addData("Status: ","Initializing Major Classes...");
    //Class Ref
    drivetrain d = new drivetrain();
    //lift l = new lift();
    Sensor S = new Sensor();
    //Final_Intake intake = new Final_Intake();

    telemetry.addData("Status: ","Setting Controller Variables...");
    //Controller
    Gamepad current;
    Gamepad previous;

    double LSx;
    double LSy;
    double RSx;
    double RSy;

    boolean up_d;
    boolean down_d;
    boolean left_d;
    boolean right_d;

    double RTrigger;
    double LTrigger;
    boolean RBumper;
    boolean LBumper;

    boolean face_a;
    boolean face_b;
    boolean face_x;
    boolean face_y;

    //VARIABLES
    double threshold;
    double threshold_;
    double trs;
    double LSx_lerped = 0;
    double RSx_lerped = 0;
    double LSy_lerped = 0;
    double Angle2;
    double rpower;
    int op;
    int DT_gear = 0;
    int DT_speed = 0;
    //double Min;
    //double Max;
    double Offset = 4390;
    int Value;
    int Dangle;

    boolean Speed;
    boolean Drive;
    double Speed1 = 0.3;
    double Speed2 = 0.5;


    double zero = 0;

    telemetry.addData("Status: ","Setting Automation Variables...");

    public String Color;
    public int Green;
    public int Blue;
    public int Red;
    public int Yellow;
    public int CurrentC;
    public ColorSensor Sensor;
    public Vector3 RGB;
    public boolean C;
    public double AVG;
    public String team;

    telemetry.addData("Status: ","Initializing Hardware....");

    map = hardwareMap;

    //l.init(map);
    d.init(map);
    intake.init(map);
    S.init(map);
    //Drive Train Motor Setup
    d.initializeMotor(d.LB, DcMotor.Direction.REVERSE);
    d.initializeMotor(d.LF, DcMotor.Direction.FORWARD);
    d.initializeMotor(d.RF, DcMotor.Direction.FORWARD);
    d.initializeMotor(d.RB, DcMotor.Direction.REVERSE);
    //Arm Motor Setup
    //l.initializeEncoderMotor(l.DC1);
    //l.initializeEncoderMotor(l.DC2);
    //Intake Setup
    intake.Initialize(zero);
     current = new Gamepad();


     previous = new Gamepad();



    //l.m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //l.m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    //Min = l.DC1.getCurrentPosition();
    //Max = Min + Offset;


    Color="";

    telemetry.addData("Status: ","Initialization Complete");
    telemetry.update();
