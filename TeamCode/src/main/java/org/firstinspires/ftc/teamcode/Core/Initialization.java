package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Libs.JCLibs.lerp;

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

@TeleOp(name="Initialization", group="demo")
@Override
public class Initialization extends PlayOpMode {
    HardwareMap map;

    //Class Ref
    drivetrain d = new drivetrain();
    //lift l = new lift();
    //Sensor S = new Sensor();
    //Final_Intake intake = new Final_Intake();

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
