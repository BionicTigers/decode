package org.firstinspires.ftc.teamcode.utils

import kotlin.math.floor

class Matrix(val value: Array<Array<Double>>) {
    operator fun get(i: Int, j: Int): Double {
        return value[i][j]
    }

    operator fun set(i: Int, j: Int, number: Number) {
        value[i][j] = number as Double
    }

    operator fun plus(other: Matrix): Matrix {
        if (this.value.size != other.value.size || this.value[0].size != other.value[0].size) {
            throw IllegalArgumentException("Matrices must have the same dimensions to be added")
        }

        val result = Array(this.value.size) { Array(this.value[0].size) { 0.0 } }

        for (i in this.value.indices) {
            for (j in this.value[i].indices) {
                result[i][j] = this.value[i][j].toDouble() + other.value[i][j].toDouble()
            }
        }

        return Matrix(result)
    }

    operator fun minus(other: Matrix): Matrix {
        if (this.value.size != other.value.size || this.value[0].size != other.value[0].size) {
            throw IllegalArgumentException("Matrices must have the same dimensions to be subtracted")
        }

        val result = Array(this.value.size) { Array(this.value[0].size) { 0.0 } }

        for (i in this.value.indices) {
            for (j in this.value[i].indices) {
                result[i][j] = this.value[i][j].toDouble() - other.value[i][j].toDouble()
            }
        }

        return Matrix(result)
    }

    operator fun times(other: Matrix): Matrix {
        if (this.value[0].size != other.value.size) {
            throw IllegalArgumentException("Number of columns of the first matrix must equal the number of rows of the second matrix")
        }

        val result = Array(this.value.size) { Array(other.value[0].size) { 0.0 } }

        for (i in this.value.indices) {
            for (j in other.value[0].indices) {
                for (k in this.value[i].indices) {
                    result[i][j] += this.value[i][k].toDouble() * other.value[k][j].toDouble()
                }
            }
        }

        return Matrix(result)
    }

    fun scalar(scalar: Double): Matrix {
        val result = Array(this.value.size) { Array(this.value[0].size) { 0.0 } }
        for (i in this.value.indices) {
            for (j in this.value[i].indices) {
                result[i][j] = this[i, j] * scalar
            }
        }
        return Matrix(result)
    }

    operator fun unaryMinus(): Matrix {
        val result = Matrix(Array(this.value.size) { Array(this.value[0].size) { 0.0 } })
        for (i in this.value.indices) {
            for (j in this.value[i].indices) {
                result[i, j] = -this.value[i][j]
            }
        }
        return result
    }

    override fun toString(): String {
        var string = "["
        for (i in this.value.indices) {
            for (j in this.value[i].indices) {
                string += this.value[i][j].toString() + ", "
            }
            string += "]\n["
        }
        string += "]"
        return string
    }

    fun Double.toHundredths(): Double {
        return floor(this * 100) / 100
    }

    fun printSimple(): String {
        var string = ""
        for(i in this.value.indices) {
            for (j in this.value[i].indices) {
                string += this[i, j].toHundredths().toString() + ", "
            }
        }
        return string
    }
}