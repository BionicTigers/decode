package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.core.commands.System
import io.github.bionictigers.axiom.core.input.ControlSchema
import io.github.bionictigers.axiom.core.input.Controllable
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.matches
import io.github.bionictigers.axiom.core.input.types.Digital
import io.github.bionictigers.axiom.core.web.Editable
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.control.PID
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.ePower
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.math.abs
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.TimeMark

class Transfer(
    hardwareMap: HardwareMap,
    val octoQuad: OctoQuad,
    telemetry: Telemetry? = null,
) : System(), Controllable<BaseProfile> {
    override val name: String = "Sorter"
    override val dependencies = listOf(octoQuad)

    enum class BallColor {
        Green,
        Purple,
        None
    }

    interface Schema : ControlSchema {
        val sort: Digital?
        val shoot: Digital?
    }

    // Indexable by (targetBay + index) % 3
//    private val balls = MutableList<BallColor>(3) { BallColor.None }

    val shootingTime = 750.milliseconds
    val intakeAngle = Angle.degrees(0.0)
    private val currentAngle: Angle
        get() = Angle.degrees((octoQuad.encoderData.position[4] / 1024.0 * 360.0))

    @Editable
    private val targetPid = PID(0.5, 0.0, 0.0, 0.0, 0.0, 360.0, -.4, .4)

    @Editable
    var reverseCorrectionWindowDegrees = 90.0

    // Hardware
    private val motor = hardwareMap.getByName<DcMotorEx>("sorter")
    private val hub = ControlHub(hardwareMap, "Control Hub")
    private val colorSensor = hardwareMap.getByName<ColorSensor>("intakeColor")

    var isShooting = false
    var shootingAt: TimeMark? = null
    var targetBay = 0
    var targetAngle = Angle.ZERO

    init {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.direction = DcMotorSimple.Direction.REVERSE
    }

    override val update = SystemCommand.continuous("Sorter Telemetry") {
        telemetry?.addData("Current Angle", currentAngle.degrees)
        telemetry?.addData("Target Angle", targetAngle.degrees)
        telemetry?.addData("Is Shooting", isShooting)
        telemetry?.addData("Power", motor.power)
    }

    override val apply = SystemCommand.continuous("Sorter Control") {
        if (isShooting) {
            if (shootingAt != null && shootingAt!!.elapsedNow() > shootingTime) {
                isShooting = false
                targetBay = (targetBay + 1) % 3
                targetAngle = getBayAngle(targetBay)
            }

            motor.ePower = -1.0
        } else {
            val forwardError = (targetAngle.degrees - currentAngle.degrees + 360.0) % 360.0
            val shortestSignedError = if (forwardError > 180.0) forwardError - 360.0 else forwardError
            val commandedError =
                if (abs(shortestSignedError) <= reverseCorrectionWindowDegrees) shortestSignedError else forwardError

            motor.ePower = targetPid.compute(0.0, commandedError)
        }
    }

    fun getBayAngle(bay: Int) = intakeAngle + Angle.degrees(bay * 120.0)

    fun sort() = SystemCommand.instant {
        isShooting = false
        targetBay = (targetBay + 2) % 3
        targetAngle = getBayAngle(targetBay)
    }

    fun shoot() = SystemCommand.instant {
        isShooting = true
        shootingAt = it.lastExecutedAt
    }

    override fun bindControls(profile: BaseProfile, gamepad: Gamepads, builder: Controls.Builder) {
        with(profile.sorter) {
            if (!desiredGamepad.matches(gamepad)) return

            sort?.let { builder.register(it) { sort() } }
            shoot?.let { builder.register(it) { shoot() } }
        }
    }
}
