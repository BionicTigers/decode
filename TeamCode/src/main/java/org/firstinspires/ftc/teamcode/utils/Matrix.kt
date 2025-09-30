package org.firstinspires.ftc.teamcode.utils

class Matrix<T: Number>(val value: Array<Array<T>>) {
    operator fun get(i: Int, j: Int): Double {
        return value[i][j] as Double
    }

    operator fun set(i: Int, j: Int, number: T) {
        value[i][j] = number
    }

    operator fun plus(other: Matrix<T>): Matrix<Double> {
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

    operator fun minus(other: Matrix<T>): Matrix<Double> {
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

    operator fun times(other: Matrix<T>): Matrix<Double> {
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

    operator fun unaryMinus() {
        for (i in this.value.indices) {
            for (j in this.value[i].indices) {
                this[i, j] = -(this.value[i][j] as Double) as T
            }
        }
    }
}