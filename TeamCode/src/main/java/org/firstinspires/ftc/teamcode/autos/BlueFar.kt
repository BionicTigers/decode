package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.scheduler.Scheduler
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.Kicker
import org.firstinspires.ftc.teamcode.mechanisms.Odometry
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.mechanisms.Sorter
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.Pose


/*
{
  "version": 1,
  "units": "mm",
  "fieldSize": {
    "width": 3657.6,
    "height": 3657.6
  },
  "startPose": {
    "xMm": 1205.693952003203,
    "yMm": 2742.2484920143665,
    "headingDeg": -90
  },
  "waypoints": {
    "Start": {
      "xMm": 1543.659364311818,
      "yMm": 3435.2183881959845,
      "headingDeg": -90
    },
    "QueueClose": {
      "xMm": 1205.693952003203,
      "yMm": 2742.2484920143665,
      "headingDeg": 180
    },
    "GrabClose": {
      "xMm": 407.64117167781785,
      "yMm": 2739.408451456245,
      "headingDeg": -180
    },
    "QueueMiddle": {
      "xMm": 1211.374042966373,
      "yMm": 2125.959690902026,
      "headingDeg": -90
    },
    "GrabMiddle": {
      "xMm": 390.60089878830775,
      "yMm": 2128.7997314601475,
      "headingDeg": -90
    },
    "ShootClose": {
      "xMm": 1830.5039579519032,
      "yMm": 1796.5149861599455,
      "headingDeg": -90
    },
    "QueueFar": {
      "xMm": 989.8504954027429,
      "yMm": 1509.6708897896856,
      "headingDeg": -90
    },
    "GrabFar": {
      "xMm": 384.9208078251378,
      "yMm": 1523.8710925802923,
      "headingDeg": -85
    }
  },
  "moves": [
    {
      "xMm": 1205.693952003203,
      "yMm": 2742.2484920143665,
      "headingDeg": -170
    },
    {
      "xMm": 407.64117167781785,
      "yMm": 2739.408451456245,
      "headingDeg": -180
    },
    {
      "xMm": 1540.819318830233,
      "yMm": 3432.378347637863,
      "headingDeg": -180
    },
    {
      "xMm": 1205.693952003203,
      "yMm": 2742.2484920143665,
      "headingDeg": 180
    },
    {
      "xMm": 1543.659364311818,
      "yMm": 3435.2183881959845,
      "headingDeg": -90
    },
    {
      "xMm": 1543.659364311818,
      "yMm": 3429.5383070797416,
      "headingDeg": -90
    },
    {
      "xMm": 1211.374042966373,
      "yMm": 2125.959690902026,
      "headingDeg": 180
    },
    {
      "xMm": 390.60089878830775,
      "yMm": 2128.7997314601475,
      "headingDeg": 20
    },
    {
      "xMm": 1830.5039579519032,
      "yMm": 1796.5149861599455,
      "headingDeg": -80
    },
    {
      "xMm": 989.8504954027429,
      "yMm": 1509.6708897896856,
      "headingDeg": -85
    },
    {
      "xMm": 384.9208078251378,
      "yMm": 1523.8710925802923,
      "headingDeg": -85
    },
    {
      "xMm": 1827.663912470318,
      "yMm": 1790.8349050437025,
      "headingDeg": -85
    },
    {
      "xMm": 1543.659364311818,
      "yMm": 3435.2183881959845,
      "headingDeg": -90
    },
    {
      "xMm": 1543.659364311818,
      "yMm": 3435.2183881959845,
      "headingDeg": -90
    },
    {
      "xMm": 1205.693952003203,
      "yMm": 2742.2484920143665,
      "headingDeg": 180
    },
    {
      "xMm": 1205.693952003203,
      "yMm": 2742.2484920143665,
      "headingDeg": 180
    },
    {
      "xMm": 1205.693952003203,
      "yMm": 2742.2484920143665,
      "headingDeg": 180
    },
    {
      "xMm": 407.64117167781785,
      "yMm": 2739.408451456245,
      "headingDeg": -180
    },
    {
      "xMm": 390.60089878830775,
      "yMm": 2128.7997314601475,
      "headingDeg": -90
    }
  ],
  "robotSize": {
    "widthMm": 457.2,
    "heightMm": 457.2
  }
}
 */


/*
    Shoot Routine:
        Move to start / close Pose
        Set Transfer to Output Position
        Spin up Shooter
        Wait for velocity to reach far / close threshold
        Repeat x3: Ignore the rotate if on final loop
        Check has Ball:
            True:
                Kick Ball
                Rotate transfer to next ball
            False:
                Rotate transfer to next ball

    Pickup Routine:
        Move to queue pose
        Set Transfer to Intake Positon
        Spin intake
        Drive at 30% speed
        Whenever color sensor reads, move to next empty slot

    Main Routine:

 */
@Autonomous
class BlueFar : LinearOpMode() {
    val startPose = Pose(1205.69, 2742.25, -90)
    val shootClosePose = Pose(1830.50, 1796.51, -45)
    val queueClosePose = Pose(1543.65, 3435.21, 0)
    val grabClosePose = Pose(407.64, 2739.40, 0)
    val queueMiddlePose = Pose(1211.37, 2125.95, 0)
    val grabMiddlePose = Pose(390.60, 2128.79, 0)
    val queueFarPose = Pose(989.85, 1509.67, 0)
    val grabFarPose = Pose(384.92, 1523.87, 0)    
    
    override fun runOpMode() {
        val odometry = Odometry(hardwareMap, telemetry, startPose)
        val drivetrain = Drivetrain(hardwareMap, telemetry, odometry)
        val intake = Intake(hardwareMap, drivetrain)
        val kicker = Kicker(hardwareMap)
        val sorter = Sorter(hardwareMap, kicker, telemetry)
        val output = Output(hardwareMap, kicker, sorter, telemetry, odometry)

        Scheduler.schedule(odometry, drivetrain, intake, kicker, sorter, output)


    }
}