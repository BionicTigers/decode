package org.firstinspires.ftc.teamcode.utils

import kotlin.time.Duration
import kotlin.time.DurationUnit

val Duration.seconds
    get() = this.toDouble(DurationUnit.SECONDS)
val Duration.milliseconds
    get() = this.toDouble(DurationUnit.MILLISECONDS)
