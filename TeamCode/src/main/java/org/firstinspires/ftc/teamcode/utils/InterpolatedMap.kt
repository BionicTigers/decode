package org.firstinspires.ftc.teamcode.utils

class InterpolatedMap : HashMap<Double, Double>() {
    override fun get(key: Double): Double {
        val points = entries.sortedBy { it.key }
        if (points.isEmpty()) return 0.0
        if (points.size == 1) return points[0].value
        if (key <= points[0].key) return points[0].value
        if (key >= points[points.size - 1].key) return points[points.size - 1].value
        for (i in 0 until points.size - 1) {
            val left = points[i]
            val right = points[i + 1]
            if (key >= left.key && key <= right.key) {
                val x1 = left.key
                val x2 = right.key
                val y1 = left.value
                val y2 = right.value
                return y1 + (key - x1) * (y2 - y1) / (x2 - x1)
            }
        }
        return 0.0
    }
}

fun interpolatedMapOf(vararg pairs: Pair<Double, Double>): InterpolatedMap {
    val map = InterpolatedMap()
    for (pair in pairs) {
        map[pair.first] = pair.second
    }
    return map
}