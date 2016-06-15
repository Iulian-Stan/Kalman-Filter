#ifndef _kalmanFilter_
#define _kalmanFilter_

#include "Eigen/Dense"

using namespace Eigen;

struct KalmanFilter
{
	///Initialize the filter with a first measurent
	KalmanFilter(
		const Vector4d& control,
		const Vector4d& first_x,
		const Matrix4d& first_p,
		const Matrix2d& measurement_noise,
		const Matrix<double, 2, 4>& observation,
		const Matrix4d& process_noise_covariance,
		const Matrix4d& state_transition
		);

	///Give the filter a measurement and input, and it will update its predictions
	void SupplyMeasurementAndCompute(const Vector2d& x);

	///Let the filter predict
	const Vector4d& Predict() const { return xPredicted; }

	///Let the filter predict
	const Matrix4d& PredictCovariance() const { return P; }

private:
	//B: control matrix: the effect of inputs on the current states
	const Vector4d B;

	//R: Estimated measurement noise: How to estimate this?
	const Matrix2d R;

	//H
	const Matrix<double, 2, 4> H;

	///The (current prediction of the) covariance
	Matrix4d P;

	//Q: Process noise covariance: How to estimate this?
	const Matrix4d Q;

	//F: state transition matrix
	const Matrix4d A;

	///The (current prediction of the) measurement
	Vector4d xPredicted;

};

#endif // _kalmanFilter_
