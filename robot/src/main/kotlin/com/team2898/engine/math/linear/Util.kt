package com.team2898.engine.math.linear

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D
import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.RealMatrix

fun rotateVector2D(source: Vector2D, rotation: Vector2D): Vector2D {
    // We need to turn the first vector into a column vector (Array2DRowRealMatrix)
    // We need to turn the second vector into a rotation matrix in the form of
    //| cos(θ) -sin(θ) |
    //| sin(θ)  cos(θ) |
    // We assume that the vectors are in the form <cos(θ), sin(θ)>
    val sourceMatrix = Array2DRowRealMatrix(source.normalize().toArray())
    val normRot = rotation.normalize()
    val rotated = Array2DRowRealMatrix(
            arrayOf(
                    doubleArrayOf(normRot.x, -normRot.y),
                    doubleArrayOf(normRot.y, normRot.x)
            )
    ).multiply(sourceMatrix)
    return Vector2D(rotated.getColumn(0)).normalize()
}

typealias Matrix = org.apache.commons.math3.linear.Array2DRowRealMatrix
// Row, col
fun RealMatrix.dim(): Pair<Int, Int> {
    return Pair(this.rowDimension, this.columnDimension)
}

operator fun RealMatrix.plus(other: RealMatrix) = this.add(other)
operator fun RealMatrix.minus(other: RealMatrix) = this.subtract(other)
operator fun RealMatrix.times(other: RealMatrix) = this.multiply(other)
operator fun RealMatrix.get(row: Int, col: Int) = this.getEntry(row, col)
operator fun RealMatrix.set(row: Int, col: Int, value: Double) = this.setEntry(row, col, value)

