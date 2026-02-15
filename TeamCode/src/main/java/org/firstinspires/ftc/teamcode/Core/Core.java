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


  //CORE VARIABLES
  DcMotor LF, LB, RF, RB;
  DcMotor GunR, GunL;
  DcMotor Intake;
  DcMotor Lift;
  //Controller
  Gamepad current;
  Gamepad previous;

  //GAMEPAD VARIABLES
  public double LSx;
  public double LSy;
  public double RSx;
  public double RSy;

  public boolean up_d;
  public boolean down_d;
  public boolean left_d;
  public boolean right_d;

  public double RTrigger;
  public double LTrigger;
  public boolean RBumper;
  public boolean LBumper;

  public boolean face_a;
  public boolean face_b;
  public boolean face_x;
  public boolean face_y;
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

  public double threshold;
  public double threshold_;
  public double trs;
  public double LSx_lerped;
  public double RSx_lerped;
  public double LSy_lerped;
  public String DriveTrainGear = "";
  int Value;

  double zero = 0;

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

  public void initHardware(HardwareMap map) {
    LF = map.get(DcMotor.class, "FrontLeft");
    RF = map.get(DcMotor.class, "FrontRight");
    LB = map.get(DcMotor.class, "BackLeft");
    RB = map.get(DcMotor.class, "BackRight");
    GunR = map.get(DcMotor.class, "GunRight");
    GunL = map.get(DcMotor.class, "GunLeft");
    Intake = map.get(DcMotor.class, "Intake");
    Lift = map.get(DcMotor.class, "Lift");
    //Sehsoru = ;
  }

  //Motor Setup
  void initializeMotor(DcMotor motor,  DcMotor.Direction direction) {
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor.setDirection(direction);
  }
  void initializeEncoderMotor(DcMotor motor,  DcMotor.Direction direction) {
    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor.setDirection(direction);
  }
  //Moves the 4 mechanum wheels in a car formation
  public void DTMove(double x, double y, double turn) {
    LF.setPower(y+x+turn);
    LB.setPower(y-x-turn);
    RF.setPower(y-x+turn);
    RB.setPower(y+x-turn);
  }
  // Gear shifter for speed setting
  public double GearShift(int DTGear) {
    double DTSpeed;
    switch (DTGear){
      case 1:
        DTSpeed = 0.2;
        DriveTrainGear = "1- 20%";
        break;
      case 2:
        DTSpeed = 0.4;
        DriveTrainGear = "2- 40%";
        break;
      case 3:
        DTSpeed = 0.6;
        DriveTrainGear = "3- 60%";
        break;
      case 4:
        DTSpeed = 0.8;
        DriveTrainGear = "4- 80%";
        break;
      case 5:
        DTSpeed = 1;
        DriveTrainGear = "5- 100%";
        break;
      default:
        DTSpeed = 0;
        DriveTrainGear ="0- 0%";
        break;
    }
    return DTSpeed;
  }
  public double GradualGearShift(double Shifting_value) {
    Shifting_value = Math.ceil(5*Shifting_value);
    return GearShift((int) Shifting_value);
  }

  public void Controller(Gamepad gamepaD){

    //Thump sticks
    LSy = gamepaD.left_stick_y;
    LSx = gamepaD.left_stick_x;
    RSx = -gamepaD.right_stick_x;
    RSy = -gamepaD.right_stick_y;

    //Dpad
    up_d = gamepaD.dpad_up;
    down_d = gamepaD.dpad_down;
    left_d = gamepaD.dpad_left;
    right_d = gamepaD.dpad_right;

    //Face buttons
    face_a = gamepaD.a;
    face_b = gamepaD.b;
    face_x = gamepaD.x;
    face_y = gamepaD.y;

    //Top buttons
    RTrigger = gamepaD.right_trigger;
    LTrigger = gamepaD.left_trigger;
    RBumper = gamepaD.right_bumper;
    LBumper = gamepaD.left_bumper;

  }

  public double applyDeadzone(double value, double threshold) {
      return Math.abs(value) > threshold ? value : 0;
  }
  public double lerpIt(double value, double factor, double threshold) {
      return lerp(value * factor, value, threshold);
  }
  public void Controlling() {

    RSy = applyDeadzone(RSy, 0.05);
    RTrigger = applyDeadzone(RTrigger, 0.05);

    LSx_lerped = lerpIt(applyDeadzone(LSx, 0.05);, 0.588, 0.8);
    LSy_lerped = lerpIt(applyDeadzone(LSy, 0.05);, 0.588, 0.8);
    RSx_lerped = lerpIt(applyDeadzone(RSx, 0.05);, 0.588, 0.8);
    DTMove(LSx_lerped, LSy_lerped, RSx_lerped);

    Intake.setPower(LSy_lerped);

    if (face_a && !previous.a) {
      GunL.setPower(-0.3);
      GunR.setPower(-0.3);
    }
    if (RTrigger != 0) {
      double power = RBumper ? RTrigger : GradualGearShift(RTrigger);
      GunL.setPower(power);
      GunR.setPower(power);
    }
    if (!face_b && previous.b) {
      Intake.setPower(0.1);
    }
    Lift.setPower(up_d ? 0.5 : down_d ? -0.5 : 0);
  }


  @Override
  protected void initialize() {

    HardwareMap map;

    telemetry.addData("Status: ","Initializing Major Classes...");
    ElapsedTime Timer = new ElapsedTime();
    //Class Ref

    map = hardwareMap;

    initHardware(map);
    //Drive Train Motor Setup
    initializeMotor(LB, DcMotor.Direction.REVERSE);
    initializeMotor(LF, DcMotor.Direction.FORWARD);
    initializeMotor(RF, DcMotor.Direction.FORWARD);
    initializeMotor(RB, DcMotor.Direction.REVERSE);
    initializeMotor(GunR, DcMotor.Direction.REVERSE);
    initializeMotor(GunL, DcMotor.Direction.FORWARD);
    initializeMotor(Intake, DcMotor.Direction.FORWARD);
    initializeEncoderMotor(Lift, DcMotor.Direction.FORWARD);
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
    telemetry.addData("Left stick x value: ",LSx);
    telemetry.addData("Left stick y value: ",LSy);
    telemetry.addData("Right stick x value: ",RSx);
    telemetry.addData("Right stick y value: ",RSy);
    telemetry.addLine();

    telemetry.addData("D-pad up on: ",up_d);
    telemetry.addData("D-pad down on: ",down_d);
    telemetry.addData("D-pad left on: ",left_d);
    telemetry.addData("D-pad right on: ",right_d);
    telemetry.addLine();

    telemetry.addData("Right Trigger value: ",RTrigger);
    telemetry.addData("Left Trigger value: ",LTrigger);
    telemetry.addData("Right Bumper on: ",RBumper);
    telemetry.addData("Left Bumper on: ",LBumper);
    telemetry.addLine();

    telemetry.addData("Face button A on: ",face_a);
    telemetry.addData("Face button B on: ",face_b);
    telemetry.addData("Face button X on: ",face_x);
    telemetry.addData("Face button Y on: ",face_y);
    telemetry.addLine();
    //VARIABLES
    //telemetry.addData("",threshold);
    //telemetry.addData("",trs);
    telemetry.addData("Left stick x value: ",LSx_lerped);
    telemetry.addData("Left stick y value: ",LSy_lerped);
    telemetry.addData("Right stick x value: ",RSx_lerped);
    telemetry.addLine();
    telemetry.addData("Gear: ",DriveTrainGear);
    telemetry.addData("",Offset);
    telemetry.addData("",Value);

    telemetry.addData("",Speed);
    telemetry.addData("",Drive);

    telemetry.addData("Status: ","Setting Automation Variables...");

    telemetry.addData("",Color);
    telemetry.addData("",Green);
    telemetry.addData("",Blue);
    telemetry.addData("",Red);
    telemetry.addData("",Yellow);
    telemetry.addData("",CurrentC);
    telemetry.addData("",Sensor);
    telemetry.addData("",RGB);
    telemetry.addData("",C);
    telemetry.addData("",AVG);
    telemetry.addData("",team);
    telemetry.update();
  }

  @Override
  protected void run(double deltaTime) throws InterruptedException {
    previous.copy(current);
    Controller(gamepad1);
    Controlling();

    Telemetry();
  }
}
