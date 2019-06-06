#include <opencv2/opencv.hpp>
using namespace cv;
#pragma comment(lib, "opencv_world346.lib")

#include <iostream>
#include <vector>
using namespace std;

#include "eqparse.h"

int main(void)
{
	vector<float> input_vec;
	
	size_t res = 100;
	
	float x_grid_max = 1.5;
	float y_grid_max = 1.5;
	float z_grid_max = 1.5;
	float x_grid_min = -x_grid_max;
	float y_grid_min = -y_grid_max;
	float z_grid_min = -z_grid_max;
	size_t x_res = res;
	size_t y_res = res;
	size_t z_res = res;
	bool make_border = true;

	float z_w = 0;
	quaternion C;
	C.x = 0.3f;
	C.y = 0.5f;
	C.z = 0.4f;
	C.w = 0.2f;
	unsigned short int max_iterations = 8;
	float threshold = 4;


	string error_string;
	quaternion_julia_set_equation_parser eqparser;
	if (false == eqparser.setup("Z = Z^2 + C", error_string, C))
	{
		cout << "Equation error: " << error_string << endl;
		return 0;
	}

	const float x_step_size = (x_grid_max - x_grid_min) / (x_res - 1);
	const float y_step_size = (y_grid_max - y_grid_min) / (y_res - 1);
	const float z_step_size = (z_grid_max - z_grid_min) / (z_res - 1);

	size_t z = 0;

	quaternion Z(x_grid_min, y_grid_min, z_grid_min, z_w);

	for (size_t z = 0; z < z_res; z++, Z.z += z_step_size)
	{
		Z.x = x_grid_min;

		for (size_t x = 0; x < x_res; x++, Z.x += x_step_size)
		{
			Z.y = y_grid_min;

			for (size_t y = 0; y < y_res; y++, Z.y += y_step_size)
			{
				vector<vector_4> points;

				float length = eqparser.iterate(points, Z, max_iterations, threshold);

				if (length < threshold)
				{
					input_vec.push_back(length);
				}
			}
		}
	}
















	float input_vec_max_val = 0;

	for (size_t i = 0; i < input_vec.size(); i++)
	{
		if (input_vec[i] > input_vec_max_val)
			input_vec_max_val = input_vec[i];
	}

	for (size_t i = 0; i < input_vec.size(); i++)
	{
		input_vec[i] /= input_vec_max_val;
		input_vec[i] *= 255.0f;
		input_vec[i] = floorf(input_vec[i]);
	}

	Mat data(1, static_cast<int>(input_vec.size()), CV_8UC1, Scalar(0));

	for (size_t i = 0; i < input_vec.size(); i++)
		data.at<unsigned char>(0, static_cast<int>(i)) = static_cast<unsigned char>(input_vec[i]);

	int histSize = 256;
	float range[] = { 0, 256 }; //the upper boundary is exclusive
	const float* histRange = { range };
	bool uniform = true, accumulate = false;
	Mat hist;
	calcHist(&data, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

	int hist_w = 600, hist_h = 600;
	int bin_w = cvRound((double)hist_w / histSize);
	Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(255, 255, 255));
	normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());

	float largest_hist = 0;
	float largest_hist_j = 0;

	for (int j = 0; j < hist.rows; j++)
	{
		for (int i = 0; i < hist.cols; i++)
		{
			if (hist.at<float>(j, i) > largest_hist)
			{
				largest_hist = hist.at<float>(j, i);
				largest_hist_j = static_cast<float>(j);
			}
		}
	}

	for (int i = 1; i < histSize; i++)
	{
		line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(hist.at<float>(i - 1))),
			Point(bin_w * (i), hist_h - cvRound(hist.at<float>(i))),
			Scalar(0, 0, 0), 1, 8, 0);
	}

	float factor = static_cast<float>(largest_hist_j * bin_w) / static_cast<float>(histImage.cols - 1);
	cout << "max value:  " << input_vec_max_val << endl;
	cout << "peak value: " << input_vec_max_val * factor << endl;


	circle(histImage, Point(static_cast<int>(largest_hist_j) * bin_w, 0), 2, Scalar(255, 127, 0), 2);


	imshow("calcHist Demo", histImage);
	waitKey();

	return 0;
}