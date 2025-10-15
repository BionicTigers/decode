package org.firstinspires.ftc.teamcode.mechanisms

import kotlin.Int
import kotlin.math.absoluteValue

class Color(val r: Int, val b: Int, val g: Int) {
    fun within(allowance: Color, otherColor: Color): Boolean {
        return (this.g - otherColor.g).absoluteValue <= allowance.g &&
        (this.b - otherColor.b).absoluteValue <= allowance.b &&
        (this.r - otherColor.r).absoluteValue <= allowance.r
    }

}
