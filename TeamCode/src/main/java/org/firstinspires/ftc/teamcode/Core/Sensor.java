package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Libs.Classes.Vector3;
import org.firstinspires.ftc.teamcode.Libs.PlayOpMode;
import static org.firstinspires.ftc.teamcode.Libs.JCLibs.lerp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Libs.Classes.GameController;
import org.firstinspires.ftc.teamcode.Libs.PlayOpMode;
public class Sensor  {

    public String Color;
    public String Team;
    public int Green;
    public  int Blue;
    public int Red;
    public int Yellow;
    public int CurrentC;
    public ColorSensor Sensor;
    public Vector3 RGB;

    public boolean C;
    public int AVG;
    public void init (HardwareMap map){
        Sensor = map.get(ColorSensor.class,"REV-31-1557");
        Color="";
    }
   public void Switch(){
        C=!C;
   }
    public void Sense() {

        CurrentC = Sensor.argb();

        Red = Sensor.red();
        Blue = Sensor.blue();
        Green = Sensor.green();
        AVG = (Red + Blue + Green) / 3;
        Yellow = (Red + Green) / 2;
        RGB = new Vector3(Red, Green, Blue);
        switch (C ? 1 : 0) {
            case 1:
                Team = "Red";
                //No Block
                if (AVG < 500) { Color = "NO Block";
                    break;}
                //Red
                if (RGB.x > RGB.z && RGB.x > RGB.y) {
                    Color = "Red";
                }
                //Yellow
                else if (Yellow > 5000) {
                    Color = "Yellow";

                }
                //Wrong Color
                else {
                    Color = "Wrong Color";

                }

                break;
            case 0:
                Team = "Blue";
                //No Block
                if (AVG < 500)

                { Color = "NO Block";
                    break;}
                //Blue
                if (RGB.z > RGB.x && RGB.z > RGB.y) {
                    Color = "Blue";

                }
                //Yellow
                else if (Yellow > 5000) {
                    Color = "Yellow";

                }
                //Wrong Color
                else {
                    Color = "Wrong Color";

                }
                break;

        }

    }
}
