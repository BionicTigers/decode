package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import io.github.bionictigers.axiom.core.commands.Command
import io.github.bionictigers.axiom.core.commands.System
import io.github.bionictigers.axiom.core.input.ControlSchema
import io.github.bionictigers.axiom.core.input.Controllable
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.matches
import io.github.bionictigers.axiom.core.input.types.Analog
import io.github.bionictigers.axiom.core.input.types.Digital
import io.github.bionictigers.axiom.core.web.Editable
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.control.PID
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.RollingAverage
import org.firstinspires.ftc.teamcode.utils.ePower
import org.firstinspires.ftc.teamcode.utils.eq
import org.firstinspires.ftc.teamcode.utils.getByName
import org.firstinspires.ftc.teamcode.utils.seconds
import kotlin.math.atan2
import kotlin.math.max

class Output(hardwareMap: HardwareMap, kicker: Kicker? = null, telemetry: Telemetry? = null, odometry: Odometry, ): System(), Controllable<BaseProfile> {
    override val name: String = "Output"

    var active: Boolean = false
    var velocity: RollingAverage = RollingAverage(5)
    var targetVelocity: Double = 0.0
    @Editable
    val pid: PID = PID(2.0, 2.0, 0.0, 1.0, 0.0, 2700.0, 0.0, 1.0)
    var maxVelocity: Double = 0.0

    interface Schema : ControlSchema {
        val shoot: Digital?
        val stop: Digital?
        val toggle: Digital?
        val toggleSlow: Digital?
        val aimLeft: Digital
        val aimRight: Digital
    }

    val motor = hardwareMap.getByName<DcMotorEx>("output")
    var angle = Angle.ZERO
    var tagAngle = 0.0

    var robotAngle = 0.0
    var targetAngle = angle.degrees - 1

    val indcLight = hardwareMap.getByName<Servo>("indcLight")
    val turret = hardwareMap.getByName<Servo>("turret")
    val hub = ControlHub(hardwareMap, "Control Hub")

    init {
        motor.direction = DcMotorSimple.Direction.FORWARD
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode == DcMotor.RunMode.RUN_WITHOUT_ENCODER
        hub.setJunkTicks()
    }
    companion object {
        var farTarget = 2700.0//1980.0
        var closeTarget = 1720.0
    }

    override val update = SystemCommand.continuous("Output Data") {
        hub.refreshBulkData()
        val lVel = hub.getEncoderTicks(0) / it.deltaTime.seconds
        tagAngle = atan2((36.54 - odometry.position.y),(0 - odometry.position.x)) + 180
        robotAngle = atan2(odometry.position.y, odometry.position.x)
        if (lVel.isFinite()) {
            velocity.plusAssign(lVel)
        }
        maxVelocity = max(velocity.average, maxVelocity)
        telemetry?.addData("maxVelocity", maxVelocity)
        telemetry?.addData("target", targetVelocity)
        telemetry?.addData("velocity", velocity.average)
        telemetry?.addData("power", motor.ePower)

        if (active) {
            if (kicker?.reset ?: false)
                motor.ePower = 1.0
            else {
                val error = targetVelocity - velocity.average
                val rampPower = if (error > 100) 1.0 else 0.0
                motor.ePower = pid.compute(velocity.average, targetVelocity) + rampPower
            }
        } else {
            motor.ePower = 0.0
        }

        if (active && velocity.average.eq(targetVelocity, 45.0))
            indcLight.position = .5
        else if (active)
            indcLight.position = .28
        else
            indcLight.position = .63


        hub.setJunkTicks()
    }

    override val apply = SystemCommand.continuous ("output data") {
        var turnAngle = tagAngle + robotAngle
        turret.position = turnAngle / 45
    }


    fun shoot() = SystemCommand.instant("Output Enable") {
//        motor.power = .83
        targetVelocity = farTarget
        active = true
    }

    fun shootClose() = SystemCommand.instant("Output Enable Slow") {
//        motor.power = .73
        targetVelocity = closeTarget
        active = true
    }

    fun stop() = SystemCommand.instant("Output Disable") {
//        motor.power = 0.0
        targetVelocity = 0.0
        active = false
    }

    fun turnLeft() = SystemCommand.instant("Output Aim Left") {
        turret.position += .1
    }

    fun turnRight() = SystemCommand.instant("Output Aim Right") {
        turret.position += -.1
    }

    override fun bindControls(profile: BaseProfile, gamepad: Gamepads, builder: Controls.Builder) {
        with(profile.output) {
            if (!desiredGamepad.matches(gamepad)) return

            shoot?.let { builder.register(it) { shoot() } }
            stop?.let { builder.register(it) { stop() } }
            toggle?.let { builder.register(it) { if (active && targetVelocity == farTarget) stop() else shoot() } }
            toggleSlow?.let { builder.register(it) { if (active && targetVelocity == closeTarget) stop() else shootClose() } }
            aimLeft?.let { builder.register(it) { turnLeft() } }
            aimRight?.let { builder.register(it) { turnRight() } }
        }
    }
}