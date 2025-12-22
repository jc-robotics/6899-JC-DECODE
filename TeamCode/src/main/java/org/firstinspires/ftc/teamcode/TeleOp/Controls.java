package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

package org.firstinspires.ftc.Initialization;

import org.firstinspires.ftc.DriveTrain;

public class Controls extends DriveTrain {

  void Controlling() {

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
