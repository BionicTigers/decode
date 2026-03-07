//package org.firstinspires.ftc.teamcode.mechanisms
//
//import com.qualcomm.robotcore.hardware.DcMotorEx
//import com.qualcomm.robotcore.hardware.HardwareMap
//import io.github.bionictigers.axiom.core.commands.System
//import io.github.bionictigers.axiom.core.input.ControlSchema
//import io.github.bionictigers.axiom.core.input.Controllable
//import io.github.bionictigers.axiom.core.input.Controls
//import io.github.bionictigers.axiom.core.input.Gamepads
//import io.github.bionictigers.axiom.core.input.matches
//import io.github.bionictigers.axiom.core.input.types.Digital
//import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware
//import org.firstinspires.ftc.teamcode.profiles.BaseProfile
//import org.firstinspires.ftc.teamcode.utils.getByName
//import java.time.InstantSource.system
//class EndGame(hardware: HardwareMap): System(), Controllable<BaseProfile> {
//
//    override val name: String = "Endgame"
//
//    interface Schema : ControlSchema {
//        val up: Digital
//        val down: Digital
//    }
//
//    val motor = hardware.getByName<DcMotorEx>("Endgame")
//
//    fun up() = SystemCommand.instant("endgame up") {
//        motor.power = 1.0
//    }
//
//    fun down() = SystemCommand.instant("endgame down") {
//        motor.power = -1.0
//    }
//
//    override fun bindControls(
//        profile: BaseProfile,
//        gamepad: Gamepads,
//        builder: Controls.Builder
//    ) {
//        TODO("Not yet implemented")
//    }
//
//
//}