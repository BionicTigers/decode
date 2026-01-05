package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.core.commands.Scheduler
import io.github.bionictigers.axiom.core.commands.groups.concurrent
import io.github.bionictigers.axiom.core.commands.groups.sequential
import org.firstinspires.ftc.teamcode.mechanisms.Kicker
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.mechanisms.Sorter
import org.firstinspires.ftc.teamcode.utils.Pose


class MilesAuton: LinearOpMode() {
    val Pattern1 = Pose(0.0,0.0,0.0)
    val Pattern2 = Pose(0.0,0.0,0.0)
    val Pattern3 = Pose(0.0,0.0,0.0)




    override fun runOpMode() {
        Scheduler.telemetry = telemetry

        val output = Output(hardwareMap, telemetry)
        val kicker = Kicker(hardwareMap, telemetry)
        val sorter = Sorter(hardwareMap, telemetry as Kicker?)


        Scheduler.schedule(output.shoot(), kicker.kick(), sorter.forward())
        Scheduler.update()


        val score = concurrent("He shoots and he scores!"){

            sequential("score preload"){
                add(output.shoot())
                add(sorter.moveForward())
                add(output.shoot())
            }
        }
    }

}