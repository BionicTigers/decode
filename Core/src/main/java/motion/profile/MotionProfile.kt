package motion.profile

import io.github.bionictigers.axiom.core.web.Display
import io.github.bionictigers.axiom.core.web.Editable

data class MotionProfile(
    @Editable @Display var jerk: Number,
    @Editable @Display var maxAcceleration: Number,
    @Editable @Display var maxVelocity: Number,
    val voltageConstant: Number? = null
) {
    @ConsistentCopyVisibility
    data class MotionResult private constructor(
        val acceleration: Double,
        val velocity: Double,
        val position: Double
    )
}