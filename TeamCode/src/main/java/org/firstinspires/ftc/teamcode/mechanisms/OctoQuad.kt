package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.core.commands.Command
import io.github.bionictigers.axiom.core.commands.System
import io.github.bionictigers.axiom.core.web.Display
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.drivers.OctoQuadFWv3
import org.firstinspires.ftc.teamcode.utils.getByName

class OctoQuad(hardwareMap: HardwareMap, val telemetry: Telemetry?): System() {
    override val name: String = "OctoQuad"

    val octoQuad = hardwareMap.getByName<OctoQuadFWv3>("octoQuad")
    private var rawEncoderData = OctoQuadFWv3.EncoderDataBlock()

    var encoderData = EncoderData()

    init {
        octoQuad.resetEverything()
    }

    override val update = SystemCommand.continuous("OctoQuad Encoder Read") {
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

    fun setEncoderDirection(index: Int, direction: OctoQuadFWv3.EncoderDirection) {
        octoQuad.setSingleEncoderDirection(index, direction)
    }

    /**
     * @param directions Index 0 is the direction for port 0, counts up from there.
     * The list can end early, but it must start at port 0. Cannot have more than 8 total elements, because there are 8 ports.
     */
    fun setManyEncoderDirections(directions: Array<OctoQuadFWv3.EncoderDirection?>) {
        require(directions.size <= 8)
        directions.forEachIndexed { index, direction ->
            if (direction != null)
                octoQuad.setSingleEncoderDirection(index, direction)
        }
    }

    /**
     * @param interval Desired sample interval in milliseconds.
     */
    fun setVelocitySampleInterval(index: Int, interval: Int) {
        octoQuad.setSingleVelocitySampleInterval(index, interval)
    }

    /**
     * @param interval Desired sample interval in milliseconds.
     */
    fun setManyVelocitySampleIntervals(range: IntRange, interval: Int) {
        for (i in range) {
            octoQuad.setSingleVelocitySampleInterval(i, interval)
        }
    }

    /**
     * @param interval Desired sample interval in milliseconds.
     */
    fun setManyVelocitySampleIntervals(vararg indices: Int, interval: Int) {
        indices.forEach {
            octoQuad.setSingleVelocitySampleInterval(it, interval)
        }
    }

    data class EncoderData(
        @Display
        var position: Array<Int> = arrayOf(0, 0, 0, 0, 0, 0, 0, 0),
        @Display
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