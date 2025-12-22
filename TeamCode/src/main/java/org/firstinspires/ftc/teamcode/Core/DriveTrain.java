package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTrain {
    DcMotor LF, LB, RF, RB, GunR, GunL, Intake;
    public void init (HardwareMap map){
        LF = map.get(DcMotor.class, "FrontLeft");
        RF = map.get(DcMotor.class, "FrontRight");
        LB = map.get(DcMotor.class, "BackLeft");
        RB = map.get(DcMotor.class, "BackRight");

        GunR = map.get(DcMotor.class, "GunRight");
        GunL = map.get(DcMotor.class, "GunLeft");

        Intake = map.get(DcMotor.class, "Intake");
    }
    //Motor Setup
    void initializeMotor(DcMotor motor,  DcMotor.Direction direction) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
    }
    void initializeEncoderMotor(DcMotor motor,  DcMotor.Direction direction) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
      switch (DTGear){
              case 1:
                  DTSpeed = 0.2;
                  DriveTrainGear = "1- 20%";
              case 2:
                  DTSpeed = 0.4;
                  DriveTrainGear = "2- 40%";
              case 3:
                  DTSpeed = 0.6;
                  DriveTrainGear = "3- 60%";
              case 4:
                  DTSpeed = 0.8;
                  DriveTrainGear = "4- 80%";
              case 5:
                  DTSpeed = 1;
                  DriveTrainGear = "5- 100%";
              default:
                  DTSpeed = 0;
                  DriveTrainGear ="0- 0%";
      }
      return DTSpeed;
      break;
    }
    public int GradualGearShift(double Shifting_value) {
      Shifting_value = Math.ceil(5*Shifting_value);
      return GearShift(Shifting_value);
    }


}


