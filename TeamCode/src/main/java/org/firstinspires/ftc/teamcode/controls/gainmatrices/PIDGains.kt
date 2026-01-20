package org.firstinspires.ftc.teamcode.controls.gainmatrices

import kotlin.math.ln
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sqrt

class PIDGains // Full constructor
// Overloaded constructors (JvmOverloads equivalent)
@JvmOverloads constructor(
    var kP: Double = 0.0,
    var kI: Double = 0.0,
    var kD: Double = 0.0,
    var maxOutputWithIntegral: Double = Double.Companion.POSITIVE_INFINITY
) {
    @JvmOverloads
    fun computeKd(gains: FeedforwardGains, percentOvershoot: Double = 0.0): PIDGains {
        this.kD = computeKd(this.kP, gains.kV, gains.kA, percentOvershoot)
        return this
    }

    companion object {
        // Static helper (top-level Kotlin function equivalent)
        fun computeKd(
            kP: Double,
            kV: Double,
            kA: Double,
            percentOvershoot: Double
        ): Double {
            val overshoot = percentOvershoot / 100.0

            val zeta: Double
            if (overshoot <= 0.0) {
                zeta = 1.0
            } else {
                zeta = -ln(overshoot) / sqrt(Math.PI.pow(2.0) + ln(overshoot).pow(2.0))
            }

            return max(
                2 * zeta * sqrt(kA * kP) - kV,
                0.0
            )
        }
    }
}