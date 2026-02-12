
package org.firstinspires.ftc.teamcode.Core;
import static org.firstinspires.ftc.teamcode.Libs.JCLibs.lerp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.*;

import org.firstinspires.ftc.teamcode.Libs.Classes.Vector3;
import org.firstinspires.ftc.teamcode.Libs.PlayOpMode;
import org.firstinspires.ftc.teamcode.Sensor;

import java.util.Objects;



@TeleOp(name="Core", group="demo")
public class Core extends PlayOpMode {

  @Override
  protected void preInitialize()
  {
    isTeleOp = true;
  }


  @Override
  protected void initialize() {

    HardwareMap map;

    telemetry.addData("Status: ","Initializing Major Classes...");
    ElapsedTime Timer = new ElapsedTime();
    //Class Ref
    DriveTrain d = new DriveTrain();
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
    ////Postive edge detectors
    //boolean face_a_current
    //boolean face_b_current
    //boolean face_x_current
    //boolean face_y_current
    ////Negative edge detectors
    //boolean face_a_previous
    //boolean face_b_previous
    //boolean face_x_previous
    //boolean face_y_previous

    //VARIABLES
    double threshold;
    double threshold_;
    double trs;
    double LSx_lerped = 0;
    double RSx_lerped = 0;
    double LSy_lerped = 0;
    //double Angle2;
    //double rpower;
    //int op;
    //int DT_gear = 0;
    //int DT_speed = 0;
    String DriveTrainGear = "";
    //double Min;
    //double Max;
    double Offset = 4390;
    int Value;

    boolean Speed;
    boolean Drive;
    double Speed1 = 0.3;
    double Speed2 = 0.5;


    double zero = 0;

    telemetry.addData("Status: ","Setting Automation Variables...");

    String Color;
    int Green;
    int Blue;
    int Red;
    int Yellow;
    int CurrentC;
    ColorSensor Sensor;
    Vector3 RGB;
    boolean C;
    double AVG;
    String team;

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
    d.initializeMotor(d.GunR, DcMotor.Direction.REVERSE);
    d.initializeMotor(d.GunL, DcMotor.Direction.FORWARD);
    d.initializeMotor(d.Intake, DcMotor.Direction.FORWARD);
    //Arm Motor Setup
    //l.initializeEncoderMotor(l.DC1);
    //l.initializeEncoderMotor(l.DC2);
    //Intake Setup
    //intake.Initialize(zero);
    current = new Gamepad();


    previous = new Gamepad();



    //l.m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //l.m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    //Min = l.DC1.getCurrentPosition();
    //Max = Min + Offset;


    Color="";

    telemetry.addData("Status: ","Initialization Complete");
    telemetry.update();
  }

  //Place any telemetry data in function
  void Telemetry(){
    //telemetry.addData("CP1", l.DC1.getCurrentPosition());
    //telemetry.addData("CP2",l.DC2.getCurrentPosition());
    //telemetry.addData("Block", S.Color);
    //telemetry.addData("Color", S.CurrentC);
    //telemetry.addData("Red", S.Red);
    //telemetry.addData("Blue", S.Blue);
    //telemetry.addData("Green", S.Green);
    //telemetry.addData("Yellow: ",S.Yellow);
    //telemetry.addData("Current Value",Value);
    //telemetry.addData("Avg C: ",S.AVG);
    //telemetry.addData("Team: ",S.Team);
    //telemetry.addLine();
    //telemetry.addLine();
    //telemetry.addData("Left Wheel Position: ", intake.lwheel.getPosition());
    //telemetry.addData("Right Wheel Position: ", intake.rwheel.getPosition());
    //telemetry.addData("Wrist Yaw Position: ", intake.wyaw.getPower());
    //telemetry.addData("Arm Yaw Position: ", intake.ayaw.getPower());
    //telemetry.addData("Arm Pitch Power: ",intake.apitch.getPower());
    telemetry.update();
  }
}

@Override
protected void run(double dt) throws InterruptedException {
  Controller(gamepad1);

  Controlling();
  //S.Sense();
  //Telemetry();
  //if(a)
  //{
  //  Value += 50;
  //}
  //if(b)
  //{
  //  Value -= 50
  //    ;
  //}
  ////Vertical
  //if (up) {
  //  l.Arm(5180,trs,2350,1);
  //}
  ////Down
  //if (down) {

  //  l.Arm(2200,trs,3200,threshold);
  //}
  ////Reset to start position
  //if(left) {
  //  l.Arm(0,trs,0,threshold);
  //}
  //// Rung 1 Part 1
  //if (right) {

  //  l.Arm(3400,trs,2350,threshold);
  //}


  // TeleOP controls





  // Sensor Code




  //if(Objects.equals(S.Color, "Wrong Color") && Objects.equals(S.Team, "Red")){

  //  intake.WheelPos(.5,.5,Servo.Direction.FORWARD,Servo.Direction.REVERSE);

  //}

  //if(Objects.equals(S.Color, "Wrong Color") && Objects.equals(S.Team, "Blue")){

  //  intake.WheelPos(.5,.5,Servo.Direction.FORWARD,Servo.Direction.REVERSE);

  //}




}
