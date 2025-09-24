package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.core.commands.System
import io.github.bionictigers.axiom.core.input.ControlSchema
import io.github.bionictigers.axiom.core.input.Controllable
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.input.Gamepads
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.getByName

class Drivetrain(hardwareMap: HardwareMap) : System(), Controllable<BaseProfile> {
    interface Schema : ControlSchema {

    }

    override val name = "Drivetrain"

    private val motors = DriveMotors(hardwareMap)

    init {
        motors.forEach {
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
//            it.zeroPowerBehavior = if (BRAKE) DcMotor.ZeroPowerBehavior.BRAKE else DcMotor.ZeroPowerBehavior.FLOAT
        }
    }

    fun stop() {
        motors.setPower(0.0, 0.0, 0.0, 0.0)
    }

    override fun bindControls(
        profile: BaseProfile,
        gamepad: Gamepads,
        builder: Controls.Builder
    ) {

    }

    private data class DriveMotors(
        val frontLeft: DcMotorEx,
        val frontRight: DcMotorEx,
        val backLeft: DcMotorEx,
        val backRight: DcMotorEx
    ): Iterable<DcMotorEx> {
        constructor(hardwareMap: HardwareMap) : this(
            hardwareMap.getByName<DcMotorEx>("frontLeft"),
            hardwareMap.getByName<DcMotorEx>("frontRight"),
            hardwareMap.getByName<DcMotorEx>("backLeft"),
            hardwareMap.getByName<DcMotorEx>("backRight")
        )

        fun setPower(fl: Double, fr: Double, bl: Double, br: Double) {
            frontLeft.power = fl
            frontRight.power = fr
            backLeft.power = bl
            backRight.power = br
        }

        override operator fun iterator() = listOf(frontLeft, frontRight, backLeft, backRight).iterator()
    }
}