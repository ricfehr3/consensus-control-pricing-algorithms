//consensus.h
// Ric Fehr March 2017
#ifndef consensus_H_   /* Include guard */
#define consensus_H_

#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

#define PI 3.141592653589793238462

double sigma_1(double z)
{
	double out = z/(sqrt(1+norm(z)*norm(z)));
	return out;
}

Mat nij(Mat qi, Mat qj, double epsilon)
{
	Mat out = (qj-qi)/(sqrt(1+epsilon*((norm(qj-qi))*(norm(qj-qi)))));
	return out;
}

double ro_h(double z, double h)
{
	double out;
	if (z>=0 && z<h)
	{
		out = 1;
	}
	else if (z>=h && z<=1)
	{
		out = (1/2)*(1+cos(PI*((z-h)/(1-h))));
	}
	else
	{
		out = 0;
	}
	return out;
}

double phi(double z, double a, double b)
{
	double c = abs(a-b)/sqrt(4*a*b);
	double out = (1/2)*((a+b)*sigma_1(z+c)+(a-b));
	return out;
}

double phi_alpha(double z, double r, double r_alpha, double d_alpha, double h, double a, double b)
{
	double out = ro_h((z/r_alpha),h)*phi((z-d_alpha),a,b);
	return out;
}

Mat sigma_1(Mat z)
{
	double tmp = norm(z, NORM_L2);
	double tmp2 = tmp*tmp;
	Mat out = z/(sqrt(1+tmp2));
	return out;
}

double sigma_norm(Mat z, double epsilon)
{
	double tmp = norm(z, NORM_L2);
	double tmp2 = tmp*tmp;
	double out = (1/epsilon)*(sqrt(1+epsilon*(tmp2))-1);
	return out;
}

double aij(Mat qi, Mat qj, double r_alpha, double h, double epsilon, int i, int j)
{
	double out;
	if (i!=j)
	{
		out = ro_h((sigma_norm((qj-qi),epsilon)/r_alpha),h);
	}
	else
	{
		out = 0;
	}
	return out;
}

Mat N_i(int i,Mat q,double r,double epsilon)
{
	int rows = q.rows;
	int cols = q.cols;;
	cv::Size s = q.size();
	rows = s.height;
	cols = s.width;
	
	double k = 0;
	double r_alpha = (1/epsilon)*(sqrt(1+epsilon*(r*r))-1); //sigma_norm(r, epsilon);

	Mat OUT = Mat(1, 4, CV_32S);
	
	for (int j=1; j<=rows; j++)
	{
		if (i!=j)
		{
			Mat TMP = Mat(1, 3, CV_64F);
			for (int h=1; h<=3; h++)
			{
				TMP.at<double>(h-1) = q.at<double>(j-1,h-1)-q.at<double>(i-1,h-1);
			} 
			double x = sigma_norm(TMP,epsilon);	
			if (x<=r_alpha)
			{
				k = k+1;
				OUT.at<int>(k-1) = j;
			}
		}
	}
	OUT = OUT.colRange(0,3);
return OUT;
}

// adding the generated adjacency matrix from the price function
Mat fi_alpha(double c1_a, double c2_a, int i, Mat q, Mat p, double r, double d, double h, double a, double b, double epsilon, Mat real_aij)
{
	int rows = q.rows;
	int cols = q.cols;

	cv::Size s = q.size();
	rows = s.height;
	cols = s.width;
	double r_alpha = (1/epsilon)*(sqrt(1+epsilon*(r*r))-1); //sigma_norm(r, epsilon);
	double d_alpha = (1/epsilon)*(sqrt(1+epsilon*(d*d))-1); //sigma_norm(r, epsilon);
	double tmp[3] = {0, 0, 0};
	Mat sum1 = Mat(1, 3, CV_64F, tmp);
	Mat sum2 = Mat(1, 3, CV_64F, tmp);
	
	Mat Ni = N_i(i,q,r,epsilon);
	int length;
	cv::Size boi = Ni.size();
	length = boi.width;

	for (int k=1; k<=length; k++)
	{
		for (int j=1; j<=rows; j++)
		{
			//cout << "dumbNi =" << endl << " " << Ni.at<int>(k-1) << endl << endl;
			//cout << "gudNi =" << endl << " " << Ni << endl << endl;
			//cout << "j =" << endl << " " << j << endl << endl;
			if(Ni.at<int>(k-1)==j)
			{
				Mat TMP1 = Mat(1, 3, CV_64FC1);
				Mat TMP2 = Mat(1, 3, CV_64FC1);
				Mat TMP3 = Mat(1, 3, CV_64FC1);
				Mat TMP4 = Mat(1, 3, CV_64FC1);
				Mat TMP5 = Mat(1, 3, CV_64FC1);
				for (int hh=1; hh<=3; hh++)
				{
					TMP1.at<double>(hh-1) = q.at<double>(j-1,hh-1)-q.at<double>(k-1,hh-1);
					TMP2.at<double>(hh-1) = q.at<double>(j-1,hh-1);
					TMP3.at<double>(hh-1) = q.at<double>(j-1,hh-1);
					TMP4.at<double>(hh-1) = p.at<double>(j-1,hh-1);
					TMP5.at<double>(hh-1) = p.at<double>(j-1,hh-1);
				}
				sum1 = phi_alpha(sigma_norm(TMP1, epsilon),r,r_alpha,d_alpha,h,a,b)*nij(TMP2,TMP3,epsilon)+sum1;
				//sum2 = aij(TMP2,TMP3,r_alpha,h,epsilon,i,j)*(TMP4-TMP3)+sum2;
				
				sum2 = real_aij.at<double>(i-1,j-1)*(TMP4-TMP3)+sum2;
				cout << real_aij.at<double>(i-1,j-1) << "\n";
				//cout << "This is the boi \t" << aij(TMP2,TMP3,r_alpha,h,epsilon,i,j) << "\n"; 
			}
		}
	}
	Mat out = c1_a*sum1+c2_a*sum2;
	return out;
}

// Collision aviodance algorithm
Mat fi_gamma(Mat q_i, Mat p_i, Mat qr, Mat pr, double c1, double c2)
{
	Mat out = Mat(1, 3, CV_64FC1);
	out = -c1*(sigma_1(q_i-qr)-c2*(p_i-pr));
	return out;
}
#endif
