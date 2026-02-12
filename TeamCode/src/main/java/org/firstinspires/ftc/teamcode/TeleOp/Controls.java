package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.Core.DriveTrain;

public class Controls extends DriveTrain {

  public void Controlling() {

    DTMove(LSx_lerped, LSy_lerped, RSx_lerped);

    Intake = LSy_lerped;

    if (current.a && !previous.a) {
      GunL = -0.3;
      GunR = -0.3;
    }
    if (RTrigger != 0) {
      GunR = GradualGearShift(RTrigger);
      GunL = GradualGearShift(RTrigger);
    }
    if (!current.a && previous.a) {
      Intake = .01;

    }
  }

}
