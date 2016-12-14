#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include "boost/filesystem.hpp"

using namespace std;

void validateArgs(int argc, char *argv[]);

int main(int argc, char *argv[])
{
	validateArgs(argc, argv);

	typedef PointMatcher<float> PM;
	typedef PM::DataPoints PointCloud;

	// Load point clouds
	const PointCloud ref(PointCloud::load(argv[1]));
	const PointCloud data(PointCloud::load(argv[2]));

	// Create the default ICP algorithm
	PM::ICP icp;

	// See the implementation of setDefault() to create a custom ICP algorithm
	icp.setDefault();

	// Compute the transformation to express data in ref
	PM::TransformationParameters T = icp(data, ref);

	// Transform data to express it in ref
	PointCloud data_out(data);
	icp.transformations.apply(data_out, T);

	// Safe files to see the results
	data_out.save("source_out.vtk");
	cout << "Final transformation:" << endl << T << endl;

	return 0;
}

void validateArgs(int argc, char *argv[])
{
	if (argc != 3)
	{
		cerr << "Wrong number of arguments, usage : " << argv[0] << " source.csv target.csv" << endl;
		cerr << "Will create 1 vtk output file: source_out.vtk" << endl;
		exit(1);
	}
}
