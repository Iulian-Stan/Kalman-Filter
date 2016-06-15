#include <fstream>
#include <iostream>
#include <iomanip>

#include "Eigen/Dense"
#include "csvIterator.h"
#include "kalmanfilter.h"

using namespace std;

int main()
{
	const double t = 1;
	const double a = .1;

    //A
	const Matrix4d state_transition = (Matrix4d() << 1.0, 0.0, t, 0.0, 0.0, 1.0, 0.0, t, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished();
    //X n-1
	const Vector4d x_first_guess (0.0, 0.0, 0.0, 0.0);
    //B
	const Vector4d control (0.5 * t * t, 0.5 * t * t, t, t);
    //P n-1
	const Matrix4d p_first_guess = (Matrix4d() << 0.25 * t * t * t * t, 0.0, 0.5 * t * t * t, 0.0, 0.0, 0.25 * t * t * t * t, 0.0, 0.5 * t * t * t, 0.5 * t * t * t, 0.0, t * t, 0.0, 0.0, 0.5 * t * t * t, 0.0, t * t ).finished() * a * a;
    //H
	const Matrix<double,2,4> observation = (Matrix<double,2,4>() << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0).finished();
    //R
	const Matrix<double,2,2> measurement_noise = (Matrix<double,2,2>() << 1.0, 0.0, 1.0, 0.0).finished();
    //Q
	const Matrix4d process_noise = p_first_guess;

	KalmanFilter k(control, x_first_guess, p_first_guess, measurement_noise, observation, process_noise, state_transition);

	ifstream file("data.csv");
    int i =0;
    for(CSVIterator loop(file);loop != CSVIterator();++loop)
    {
		//Perform a noisy measurement
		const Vector2d z_measured ((*loop)[1], (*loop)[0]);
		//Pass this measurement to the filter

		k.SupplyMeasurementAndCompute(z_measured);
		//Display what the filter predicts
		const Vector4d x_est_last = k.Predict();

		cout << right << setw(3) << (*loop)[1] << " - " << setw(3) << (*loop)[0] //input data
			<< "  -> " 
			<< setw(8) << x_est_last(0) << " - " << setw(8) << x_est_last(1) //filter estimations
			<< "\t" << x_est_last(2) << "   " << x_est_last(3) << '\n';
		++i;
	}
	getchar();
	return 0;
}
