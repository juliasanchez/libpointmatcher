// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2012,
François Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
You can contact the authors at <f dot pomerleau at gmail dot com> and
<stephane at magnenat dot net>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include "boost/filesystem.hpp"

using namespace std;

/**
  * Code example for ICP taking 2 points clouds (2D or 3D) relatively close 
  * and computing the transformation between them.
  *
  * Instead of using yaml file for configuration, we configure the solution
  * directly in the code.
  *
  * This code replicate the solution in /evaluations/official_solutions/Besl92_pt2point.yaml
  */
int main(int argc, char *argv[])
{
	validateArgs(argc, argv);
	
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;

	// Load point clouds
	const DP ref(DP::load(argv[1]));
	const DP data(DP::load(argv[2]));

	// Create the default ICP algorithm
	PM::ICP icp;
	PointMatcherSupport::Parametrizable::Parameters params;
	std::string name;
	
	// Uncomment for console outputs
	setLogger(PM::get().LoggerRegistrar.create("FileLogger"));

	// Prepare reading filters
	name = "MinDistDataPointsFilter";
	params["minDist"] = "1.0";
	PM::DataPointsFilter* minDist_read = 
		PM::get().DataPointsFilterRegistrar.create(name, params);
	params.clear();

	name = "RandomSamplingDataPointsFilter";
	params["prob"] = "0.05";
	PM::DataPointsFilter* rand_read = 
		PM::get().DataPointsFilterRegistrar.create(name, params);
	params.clear();

	// Prepare reference filters
	name = "MinDistDataPointsFilter";
	params["minDist"] = "1.0";
	PM::DataPointsFilter* minDist_ref = 
		PM::get().DataPointsFilterRegistrar.create(name, params);
	params.clear();

	name = "RandomSamplingDataPointsFilter";
	params["prob"] = "0.05";
	PM::DataPointsFilter* rand_ref = 
		PM::get().DataPointsFilterRegistrar.create(name, params);
	params.clear();

	// Prepare matching function
	name = "KDTreeMatcher";
	params["knn"] = "1";
	params["epsilon"] = "3.16";
	PM::Matcher* kdtree = 
		PM::get().MatcherRegistrar.create(name, params);
	params.clear();

	// Prepare outlier filters
	name = "TrimmedDistOutlierFilter";
	params["ratio"] = "0.75";
	PM::OutlierFilter* trim = 
		PM::get().OutlierFilterRegistrar.create(name, params);
	params.clear();

	// Prepare error minimization
	name = "PointToPointErrorMinimizer";
	PM::ErrorMinimizer* pointToPoint =   
		PM::get().ErrorMinimizerRegistrar.create(name);

	// Prepare transformation checker filters
	name = "CounterTransformationChecker";
	params["maxIterationCount"] = "150";
	PM::TransformationChecker* maxIter = 
		PM::get().TransformationCheckerRegistrar.create(name, params);
	params.clear();

	name = "DifferentialTransformationChecker";
	params["minDiffRotErr"] = "0.001";
	params["minDiffTransErr"] = "0.01";
	params["smoothLength"] = "4";
	PM::TransformationChecker* diff = 
		PM::get().TransformationCheckerRegistrar.create(name, params);
	params.clear();

	// Prepare inspector
	PM::Inspector* nullInspect =
		PM::get().InspectorRegistrar.create("NullInspector");

	//name = "VTKFileInspector";
    //	params["dumpDataLinks"] = "1"; 
    //	params["dumpReading"] = "1"; 
    //	params["dumpReference"] = "1"; 

	//PM::Inspector* vtkInspect =
	//	PM::get().InspectorRegistrar.create(name, params);
	params.clear();
	
	// Prepare transformation
	PM::Transformation* rigidTrans =
		PM::get().TransformationRegistrar.create("RigidTransformation");
	
	// Build ICP solution
	icp.readingDataPointsFilters.push_back(minDist_read);
	icp.readingDataPointsFilters.push_back(rand_read);

	icp.referenceDataPointsFilters.push_back(minDist_ref);
	icp.referenceDataPointsFilters.push_back(rand_ref);

	icp.matcher.reset(kdtree);
	
	icp.outlierFilters.push_back(trim);
	
	icp.errorMinimizer.reset(pointToPoint);

	icp.transformationCheckers.push_back(maxIter);
	icp.transformationCheckers.push_back(diff);
	
	// toggle to write vtk files per iteration
	icp.inspector.reset(nullInspect);
	//icp.inspector.reset(vtkInspect); 

	icp.transformations.push_back(rigidTrans);

	// Compute the transformation to express data in ref
	PM::TransformationParameters T = icp(data, ref);

	// Transform data to express it in ref
	DP data_out(data);
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
