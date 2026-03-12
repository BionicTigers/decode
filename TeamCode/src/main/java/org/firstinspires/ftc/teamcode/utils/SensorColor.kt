package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.ColorSensor
import kotlin.math.sqrt

data class SensorColor(val red: Double, val green: Double, val blue: Double) {
    companion object {
        val BLACK = SensorColor(0.0, 0.0, 0.0)

        fun fromSensor(sensor: ColorSensor): SensorColor {
            val argb = sensor.argb()
            val a = (argb shr 24) and 0xFF
            val r = (argb shr 16) and 0xFF
            val g = (argb shr 8) and 0xFF
            val b = argb and 0xFF
            return SensorColor(
                r.toDouble(),
                g.toDouble(),
                b.toDouble(),
            )
        }
    }

    enum class Channel {
        Red,
        Green,
        Blue,
        None,
    }

    val highestChannelValue: Double
        get() = maxOf(red, green, blue)

    val dominantChannel: Channel
        get() = when {
            highestChannelValue <= 0.0 -> Channel.None
            red == highestChannelValue -> Channel.Red
            green == highestChannelValue -> Channel.Green
            else -> Channel.Blue
        }

    fun normalized(): SensorColor {
        val highestChannel = highestChannelValue
        return if (highestChannel <= 0.0) {
            BLACK
        } else {
            SensorColor(
                red = red / highestChannel,
                green = green / highestChannel,
                blue = blue / highestChannel,
            )
        }
    }

    fun distanceTo(other: SensorColor): Double {
        val normalizedColor = normalized()
        val normalizedOther = other.normalized()

        val redDifference = normalizedColor.red - normalizedOther.red
        val greenDifference = normalizedColor.green - normalizedOther.green
        val blueDifference = normalizedColor.blue - normalizedOther.blue

        return sqrt(
            redDifference * redDifference +
                greenDifference * greenDifference +
                blueDifference * blueDifference,
        )
    }

    fun isInRange(minColor: SensorColor, maxColor: SensorColor, normalize: Boolean = true): Boolean {
        val color = if (normalize) normalized() else this
        val lowerBound = if (normalize) minColor.normalized() else minColor
        val upperBound = if (normalize) maxColor.normalized() else maxColor

        return color.red in minOf(lowerBound.red, upperBound.red)..maxOf(lowerBound.red, upperBound.red) &&
            color.green in minOf(lowerBound.green, upperBound.green)..maxOf(lowerBound.green, upperBound.green) &&
            color.blue in minOf(lowerBound.blue, upperBound.blue)..maxOf(lowerBound.blue, upperBound.blue)
    }
}
