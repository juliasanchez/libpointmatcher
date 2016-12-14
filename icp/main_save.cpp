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
    ifstream ifs("icp_configuration.yaml");
    icp.loadFromYaml(ifs);

    // Compute the transformation to express data in ref
    PM::TransformationParameters T = icp(data, ref);

    // Transform data to express it in ref
    PointCloud data_out(data);
    icp.transformations.apply(data_out, T);

    // Safe files to see the results
    ref.save("test_ref.vtk");
    data.save("test_data_in.vtk");
    data_out.save("test_data_out.vtk");
    cout << "Final transformation:" << endl << T << endl;

    return 0;
}

void validateArgs(int argc, char *argv[])
{
    if (argc != 3)
    {
        cerr << "Wrong number of arguments, usage " << argv[0] << " reference.csv reading.csv" << endl;
        cerr << "Will create 3 vtk files for inspection: ./test_ref.vtk, ./test_data_in.vtk and ./test_data_out.vtk" << endl;
        exit(1);
    }
}
