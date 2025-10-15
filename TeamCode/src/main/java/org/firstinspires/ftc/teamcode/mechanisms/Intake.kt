package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.robocol.Command
import io.github.bionictigers.axiom.core.commands.BaseCommand
import io.github.bionictigers.axiom.core.input.ControlSchema
import io.github.bionictigers.axiom.core.input.Controllable
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.types.Digital
import io.github.bionictigers.axiom.core.commands.System
import org.firstinspires.ftc.teamcode.profiles.BaseProfile

import org.firstinspires.ftc.teamcode.utils.getByName

class MilesIntake(hardwareMap: HardwareMap): System(), Controllable<BaseProfile> {
    interface Schema : ControlSchema{
        val suckIn: Digital
        val suckOut: Digital
    }
    val color = Color(4,9,5)
    val purple = Color(0,0,0)
    val green = Color(0,0,0)
    val allowance = Color(0,0,0)
    var purpleBalls = 0
    var greenBalls = 0
    override val name = "intake"
    val intakeColor = hardwareMap.getByName<ColorSensor>("IntakeColor")
    val intake = hardwareMap.getByName<DcMotorEx>("OverheadIntake")
    fun suckIn(): BaseCommand = SystemCommand.instant { intake.power = 1.0  }
    fun suckOut(): BaseCommand = SystemCommand.instant { intake.power = -1.0 }
    fun seeColor(){
        if (color.within(allowance, green)) {
            greenBalls++
            suckIn()
        }
        if (color.within(allowance, purple)) {
            purpleBalls++
            suckIn()
        }
    }
    override fun bindControls(
        profile: BaseProfile,
        gamepad: Gamepads,
        builder: Controls.Builder)
    {
            profile.milesIntake.suckIn.let { builder.register(it) { suckIn() } }
            profile.milesIntake.suckOut.let { builder.register(it) { suckOut() } }
    }
}