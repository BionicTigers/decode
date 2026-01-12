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
import org.firstinspires.ftc.teamcode.teleops.Drive
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.RollingAverage
import org.firstinspires.ftc.teamcode.utils.ePower
import org.firstinspires.ftc.teamcode.utils.eq
import org.firstinspires.ftc.teamcode.utils.getByName
import org.firstinspires.ftc.teamcode.utils.seconds
import java.lang.Math.PI
import java.lang.StrictMath.PI
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sqrt

class Output(hardwareMap: HardwareMap, kicker: Kicker? = null, telemetry: Telemetry? = null, odometry: Odometry, ): System(), Controllable<BaseProfile> {
    override val name: String = "Output"

    var active: Boolean = false
    val verticalAcceleration: Double = -9.81
    val horizontalAcceleration: Double = 0.0
    var velo = 0.0
    var time: Double = 0.0
    var motorPower: Double = 0.0
    val c = 2* kotlin.math.PI * (9.72 / 2)
    var RPM: Double = 0.0
    val initialVerticalVelocity: Double = 0.0
    var finalVeriticalVelocity: Double = 0.0
    var wheelSpeedVar: Double = 0.0
    var initialHorizontalVelocity: Double = 0.0
    var finalHorizontalVelocity: Double = 0.0
    var distance: Double = 0.0
    var velocity: RollingAverage = RollingAverage(5)
    var targetVelocity: Double = 0.0
    var startingAngle = odometry.position.rotation.degrees
    @Editable
    val pid: PID = PID(100.0, 2.0, 0.0, 1.0, 0.0, 1980.0, 0.0, 1.0)
    var maxVelocity: Double = 0.0
    var servoMaxPosition: Double = 0.45
    interface Schema : ControlSchema {
        val shoot: Digital?
        val stop: Digital?
        val toggle: Digital?
        val toggleSlow: Digital?
    }

    val motor = hardwareMap.getByName<DcMotorEx>("output")
    val servo = hardwareMap.getByName<Servo>("pivot")
    var angle = Angle.ZERO
    var tagAngle = Angle.radians(0.0).degrees

    var dx = -(0 - odometry.position.x)   // flip X
    var dy =  3654 - odometry.position.y

    var correctedAngle = 0.0

    var robotAngle = odometry.position.rotation       //Angle.radians(0.0).degrees
    val indcLight = hardwareMap.getByName<Servo>("indcLight")
    val hub = ControlHub(hardwareMap, "Control Hub")

    init {
        var startHeading = odometry.position.rotation.degrees  //

        motor.direction = DcMotorSimple.Direction.REVERSE
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode == DcMotor.RunMode.RUN_WITHOUT_ENCODER
        hub.setJunkTicks()
    }
    companion object {
        var farTarget = 1980.0
        var closeTarget = 1720.0
    }

    override val update = SystemCommand.continuous("Output Data") {
        distance = sqrt((3654.0 - odometry.position.x).pow(2)+(0-odometry.position.y).pow(2))
        finalVeriticalVelocity = sqrt(initialVerticalVelocity.pow(2) + 2 * verticalAcceleration * 15)
        time = (finalVeriticalVelocity - initialVerticalVelocity) / verticalAcceleration

        initialHorizontalVelocity = distance/time
        initialHorizontalVelocity =  finalHorizontalVelocity

        velo = sqrt(finalHorizontalVelocity.pow(2)+ finalVeriticalVelocity.pow(2))
        wheelSpeedVar = 2 * kotlin.math.PI * ((9.72 / 2.0) + 5/(9.72 / 2.0)) * velo

        RPM = (wheelSpeedVar / c) * 60/6000


        correctedAngle =  odometry.position.rotation.degrees - startingAngle
        hub.refreshBulkData()
        val lVel = hub.getEncoderTicks(0) / it.deltaTime.seconds
        dx = -(0 - odometry.position.x)   // flip X
        dy =  3654 - odometry.position.y
        tagAngle = Math.toDegrees(atan2(dy, dx))

        robotAngle = Angle.radians(atan2(odometry.position.y,-odometry.position.x))
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
        var turnAngle = normalizeAngle(tagAngle - robotAngle.degrees)
        var middleSpot: Double = 0.5
        servo.position = (middleSpot + (turnAngle / 45) * servoMaxPosition).coerceIn(0.0,1.0)


        telemetry?.addData("servo position",   (turnAngle / 45) * servoMaxPosition)
        telemetry?.addData("tagangle", tagAngle )
        telemetry?.addData("robotangle", robotAngle.degrees )
        telemetry?.addData("is it right?", tagAngle - robotAngle.degrees)
    }

    fun normalizeAngle(angle: Double): Double{
        var a: Double = angle
        while (a > 180) a -= 360
        while (a < -180) a += 360
        return a
    }
    fun shoot() = SystemCommand.instant("Output Enable") {
////        motor.power = .83
//        targetVelocity = farTarget
//        active = true
        motor.power = RPM
    }

//    fun shootClose() = SystemCommand.instant("Output Enable Slow") {
////        motor.power = .73
//        targetVelocity = closeTarget
//        active = true
//    }

    fun stop() = SystemCommand.instant("Output Disable") {
//        motor.power = 0.0
        targetVelocity = 0.0
        active = false
    }




    override fun bindControls(profile: BaseProfile, gamepad: Gamepads, builder: Controls.Builder) {
        with(profile.output) {
            if (!desiredGamepad.matches(gamepad)) return

            shoot?.let { builder.register(it) { shoot() } }
            stop?.let { builder.register(it) { stop() } }
            toggle?.let { builder.register(it) { if (active && targetVelocity == farTarget) stop() else shoot() } }

        }
    }
}