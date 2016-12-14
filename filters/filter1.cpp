#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include <fstream>

using namespace std;
using namespace PointMatcherSupport;

typedef PointMatcher<float> PM;
typedef PM::DataPoints PointCloud;

int main (int argc, char** argv)
{

  if (argc < 4 )
	{
		std::cout << "Usage: "<<argv[0]<<" configuration_yalm_file cloud_file.csv output_name.csv "<< std::endl<< std::endl;
		exit(EXIT_FAILURE);
	}

	if ( argc > 4 )
	{
		std::cout << "TOO MANY ARGUMENTS"<< std::endl<< std::endl;
		exit(EXIT_FAILURE);
	}

  setLogger(PM::get().LoggerRegistrar.create("FileLogger"));
  PointCloud cloud(PointCloud::load(argv[2]));

  ifstream file(argv[1]);
  PM::DataPointsFilters filter(file);
  filter.apply(cloud);

  cloud.save(argv[3]);

  return 0;
}
