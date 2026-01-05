package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import io.github.bionictigers.axiom.core.commands.BaseCommandState
import io.github.bionictigers.axiom.core.commands.Command
import io.github.bionictigers.axiom.core.commands.System
import io.github.bionictigers.axiom.core.input.ControlSchema
import io.github.bionictigers.axiom.core.input.Controllable
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.types.Digital
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.control.KalmanFilter
import org.firstinspires.ftc.teamcode.control.PID
import org.firstinspires.ftc.teamcode.motion.limeLight
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.getByName
import org.firstinspires.ftc.teamcode.utils.seconds
import kotlin.math.absoluteValue

class Output(hardwareMap: HardwareMap, telemetry: Telemetry): System(), Controllable<BaseProfile> {
    override val name: String = "Output"

    interface Schema : ControlSchema {
        val shoot: Digital
        val stop: Digital
        val speedUp: Digital
        val speedDown: Digital
    }

    val motor = hardwareMap.getByName<DcMotorEx>("output")
    val servo = hardwareMap.getByName<Servo>("spiny thing")
    val hub = ControlHub(hardwareMap,"controlhub")

    val pid = PID(1.0, 0.0, 0.0, 0.0, -200, 200, -1, 1)

    init {
        motor.direction = DcMotorSimple.Direction.REVERSE
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        hub.setJunkTicks(2)
        pid.reset()
    }

    var state = StateOutput(0.7,0.7,0.7,0.7,false)

    override val beforeRun = SystemCommand.continuous("Sorter Update", state) {
        hub.refreshBulkData()

        val deltaTicks = hub.getEncoderTicks(3)

        it.angle -= (deltaTicks / 2000) * 360.0

        if (it.angle < 0) {
            it.angle += 360.0
        }

        it.velocity = deltaTicks / it.deltaTime.seconds

        it.angle %= 360.0

        if (it.isStopped == true){
            motor.power  = 0.0
        }else {
            if ((it.velocity - it.targetVelocity).absoluteValue >= 0.0) {
                motor.power = pid.compute(it.velocity, it.targetVelocity)
            }
        }
    }



    fun shoot() = SystemCommand.instant("turn on output", state) {
        it.isStopped = true
    }

    fun stop() = SystemCommand.instant("turn on output", state) {
        it.isStopped = false
    }

    fun speedUp() = Command.instant("output increase",state) {
       it.velocity++
    }

    fun slowDown() = Command.instant("output decrease",state) {
        it.velocity--
    }

    fun turnTheSpinyThingy() = Command.continuous("output spin", state)  {

    }

    override fun bindControls(profile: BaseProfile, gamepad: Gamepads, builder: Controls.Builder) {
        //with(profile.output) {
         //   builder.register(shoot) { shoot() }
           // builder.register(stop) { stop() }
           // builder.register(speedUp) { speedUp() }
            //builder.register(speedDown) { slowDown() }
        }
    }
    data class StateOutput(
        var speed: Double = 0.9,
        var velocity: Double = 0.0,
        var angle: Double = 0.0,
        var targetVelocity: Double = 0.0,
        var isStopped: Boolean = false,

       // var kalmanFilter: KalmanFilter,


    ): BaseCommandState()
