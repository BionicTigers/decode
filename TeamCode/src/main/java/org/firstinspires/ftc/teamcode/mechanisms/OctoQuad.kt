package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.core.commands.Command
import io.github.bionictigers.axiom.core.commands.System
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.drivers.OctoQuadFWv3
import org.firstinspires.ftc.teamcode.utils.getByName

class OctoQuad(hardwareMap: HardwareMap, val telemetry: Telemetry?): System() {
    override val name: String = "OctoQuad"

    val octoQuad = hardwareMap.getByName<OctoQuadFWv3>("octoQuad")
    private val rawEncoderData = OctoQuadFWv3.EncoderDataBlock()

    var encoderData = EncoderData()

    override val update = Command.create("OctoQuad Encoder Read") {
        octoQuad.readAllEncoderData(rawEncoderData)
        if (rawEncoderData.crcOk) {
            for (i in 0..7) {
                encoderData.position[i] = rawEncoderData.positions[i]
                encoderData.velocity[i] = rawEncoderData.velocities[i].toDouble()
            }
        } else {
            println("CRC not ok :(")
            telemetry?.addLine("CRC not ok :(")
        }
    }

    data class EncoderData(
        var position: Array<Int> = arrayOf(0, 0, 0, 0, 0, 0, 0, 0),
        var velocity: Array<Double> = arrayOf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    ) {
        override fun equals(other: Any?): Boolean {
            if (this === other) return true
            if (javaClass != other?.javaClass) return false

            other as EncoderData

            if (!position.contentEquals(other.position)) return false
            if (!velocity.contentEquals(other.velocity)) return false

            return true
        }

        override fun hashCode(): Int {
            var result = position.contentHashCode()
            result = 31 * result + velocity.contentHashCode()
            return result
        }
    }
}