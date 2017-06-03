// adjacency.h
// Ric Fehr, March 2017
#ifndef adjacency_H_   /* Include guard */
#define adjacency_H_

#include <iostream>

// Generate factorial 
double myFact(int N)
{
	int foo = 1;
	for (int i=1; i<=N; i++)
	{
		foo = foo*i;
	}
	//cout << foo << "\n";
	return foo;
}

double combo(int N, int K)
{
	return (myFact(N))/(myFact(K)*myFact(N-K));
}

template<size_t N>
double getPMax(double (&tmp_price)[N], double n_agents)
{
	// Find how many elements are in price array
	int size = sizeof(tmp_price)/sizeof(tmp_price[0]);
	
	// Bubble sort that bad boi
	// get that temp variable for the swapping
	double swap;
	for (int c = 0 ; c < ( size - 1 ); c++)
	{
		for (int d = 0 ; d < size - c - 1; d++)
		{
			if (tmp_price[d] > tmp_price[d+1]) /* For decreasing order use < */
			{
				swap = tmp_price[d];
				tmp_price[d] = tmp_price[d+1];
				tmp_price[d+1] = swap;
			}
		}
	}
	
	int N_snd = n_agents-1;
	int min_num_connect = combo(N_snd,2);
	
	double max_price = tmp_price[min_num_connect];
	
	return max_price;
}

Mat getAdj(Mat P, double maxP, int n)
{
	// create connectivity matrix
	Mat C = Mat(n, n, CV_64FC1);
	
	// generate adjacency matrix
	Mat A = Mat(n, n, CV_64FC1);
	
	// generated communication quality matrix
	Mat G = Mat(n, n, CV_64FC1);
	
	// generate connectivity matrix according to pmax function
	for (int i=0; i<n; i++)
	{
		for(int j=0; j<n; j++)
		{
			if(i==j)
			{
				C.at<double>(i,j) = 0;
			}
			else if(P.at<double>(i,j) > maxP)
			{
				C.at<double>(i,j) = 0;
			}
			else if(P.at<double>(i,j) <= maxP)
			{
				C.at<double>(i,j) = 1;
				C.at<double>(j,i) = 1;
			}
			
			// create communication quality from price
			G.at<double>(i,j) = 1/P.at<double>(i,j);
		}
	}
	
	for (int i=0; i<n; i++)
	{
		for (int j=0; j<n; j++)
		{
			A.at<double>(i,j) = G.at<double>(i,j)*C.at<double>(i,j); 
		}
	}
	
	cout << C << "\n";
	
	return A;
}

#endif
