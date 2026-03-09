//package org.firstinspires.ftc.teamcode.mechanisms
//
//import com.qualcomm.robotcore.hardware.DcMotorEx
//import com.qualcomm.robotcore.hardware.DcMotorSimple
//import com.qualcomm.robotcore.hardware.HardwareMap
//import io.github.bionictigers.axiom.core.commands.Command
//import io.github.bionictigers.axiom.core.commands.System
//import io.github.bionictigers.axiom.core.input.ControlSchema
//import io.github.bionictigers.axiom.core.input.Controllable
//import io.github.bionictigers.axiom.core.input.Controls
//import io.github.bionictigers.axiom.core.input.Gamepads
//import io.github.bionictigers.axiom.core.input.matches
//import io.github.bionictigers.axiom.core.input.types.Digital
//import io.github.bionictigers.axiom.core.web.Editable
//import org.firstinspires.ftc.teamcode.control.PID
//import org.firstinspires.ftc.teamcode.drivers.OctoQuadFWv3
//import org.firstinspires.ftc.teamcode.profiles.BaseProfile
//import org.firstinspires.ftc.teamcode.utils.RollingAverage
//import org.firstinspires.ftc.teamcode.utils.ePower
//import org.firstinspires.ftc.teamcode.utils.getByName
//import kotlin.math.abs
//
//class MotorRun(hardwareMap: HardwareMap): System(), Controllable<BaseProfile> {
//    override val name: String = "mtoro"
//
//    interface Schema : ControlSchema {
//        val stop: Digital?
//        val start: Digital?
//    }
//
//    val motor = hardwareMap.getByName<DcMotorEx>("motor")
//
//    override val apply = Command.continuous {
//        motor.power = 1.0
//    }
//
//    fun on() = SystemCommand.instant("Motor Enable") {
//        motor.power = 1.0
//    }
//
//    fun off() = SystemCommand.instant("Motor Disable") {
//        motor.power = 0.0
//    }
//
//    override fun bindControls(profile: BaseProfile, gamepad: Gamepads, builder: Controls.Builder) {
//        with(profile.motor) {
//            if (!desiredGamepad.matches(gamepad)) return
//            start?.let { builder.register(it) { on() } }
//            stop?.let { builder.register(it) { off() } }
//        }
//    }
//}