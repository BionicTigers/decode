package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.bionictigers.axiom.core.commands.Scheduler
import io.github.bionictigers.axiom.core.input.Controls
import org.firstinspires.ftc.teamcode.control.KalmanFilter
import org.firstinspires.ftc.teamcode.mechanisms.LimeLight
import org.firstinspires.ftc.teamcode.motion.Odometry
import org.firstinspires.ftc.teamcode.utils.Angle
import java.util.ResourceBundle


@TeleOp(name = "Teleop")
class LimeTeleop: LinearOpMode() {
    override fun runOpMode() {
        Scheduler.reset()

        var deltaAngle: Angle = Angle.radians(0)

        val limeLight = LimeLight(hardwareMap, telemetry)
        val odometry = Odometry(hardwareMap, telemetry)
        val KalmanFilter = KalmanFilter(hardwareMap, telemetry) { deltaAngle }

        Scheduler.telemetry = telemetry
        Scheduler.schedule(limeLight, odometry)

        waitForStart()

        while (opModeIsActive()) {
            deltaAngle = Angle.degrees(gamepad1.left_stick_x * 300)

            KalmanFilter.prediction()
            KalmanFilter.update()

            Scheduler.update()
            telemetry.update()
        }

        Scheduler.reset()
    }


}