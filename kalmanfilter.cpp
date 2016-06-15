#ifdef _WIN32
#undef __STRICT_ANSI__
#endif

#include "kalmanfilter.h"

KalmanFilter::KalmanFilter(
	const Vector4d& control,
	const Vector4d& first_x,
	const Matrix4d& first_p,
	const Matrix2d& measurement_noise,
	const Matrix<double,2,4>& observation,
	const Matrix4d& process_noise_covariance,
	const Matrix4d& state_transition)
	: B(control),
	R(measurement_noise),
	H(observation),
	P(first_p),
	Q(process_noise_covariance),
	A(state_transition),
	xPredicted(first_x)
{}

void KalmanFilter::SupplyMeasurementAndCompute(const Vector2d& x)
{
	/// 1/7) State prediction
	const Vector4d x_current
		= A * xPredicted +
		B * 0;
	/// 2/7) Covariance prediction
	const Matrix4d p_current
		= A * P * A.transpose() + Q;
	/// 3/7) Innovation (y with a squiggle above it)
	const Vector2d z_measured = x; //x is noisy
	const Vector2d y = z_measured - H * x_current;
	/// 4/7) Innovation covariance (S)
	const Matrix2d S = H * p_current * H.transpose() + R;
	/// 5/7) Kalman gain (K)
	const Matrix<double,4,2> kalman_gain = p_current * H.transpose() * S.inverse();
	/// 6/7) Update state prediction
	xPredicted = x_current + kalman_gain * y;
	/// 7/7) Update covariance prediction
    P = (Matrix4d::Identity() - kalman_gain * H) * p_current;
}
