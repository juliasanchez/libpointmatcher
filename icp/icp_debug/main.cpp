#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include "boost/filesystem.hpp"
#include <fstream>
#include <algorithm>

using namespace std;
using namespace PointMatcherSupport;

typedef PointMatcher<float> PM;
typedef PM::DataPoints PointCloud;
typedef PM::Parameters Parameters;

int validateArgs(const int argc, const char *argv[],bool& isTransfoSaved,string& configFile,string& initTranslation, string& initRotation);
PM::TransformationParameters parseTranslation(string& translation,const int cloudDimension);
PM::TransformationParameters parseRotation(string& rotation,const int cloudDimension);
void usage(const char *argv[]);

int main(int argc, const char *argv[])
{
    bool isTransfoSaved = false;
    string configFile;
    string initTranslation("0,0,0");
    string initRotation("1,0,0;0,1,0;0,0,1");
    const int ret = validateArgs(argc, argv, isTransfoSaved, configFile, initTranslation, initRotation);

    const char *src_file(argv[argc-2]);
    const char *tgt_file(argv[argc-1]);

    // Load point clouds
    const PointCloud source(PointCloud::load(src_file));
    const PointCloud target(PointCloud::load(tgt_file));

    PM::ICP icp;

    if (configFile.empty())
    {
        icp.setDefault();
    }
    else
    {
        // load YAML config
        ifstream ifs(configFile.c_str());
        icp.loadFromYaml(ifs);
    }

    int cloudDimension = target.getEuclideanDim();

    if (!(cloudDimension == 3))
    {
        cerr << "Invalid input point clouds dimension" << endl;
        exit(1);
    }


    //PROBLEME RESOLU DE MANNIERE ABSURDE :
    PM::TransformationParameters translation1 = parseTranslation(initTranslation, cloudDimension);
    Eigen::Matrix<float, 4, 4> translation=translation1;
    PM::TransformationParameters rotation1 = parseRotation(initRotation, cloudDimension);
    Eigen::Matrix<float, 4, 4> rotation=rotation1;
    PM::TransformationParameters initTransfo = translation*rotation;

    PM::Transformation* rigidTrans;
    rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

    if (!rigidTrans->checkParameters(initTransfo))
    {
        cerr << endl<< "Initial transformation is not rigid, identiy will be used"<< endl;
        initTransfo.setIdentity(4,4);
    }

    const PointCloud initializedSource = rigidTrans->compute(source, initTransfo);

    PM::TransformationParameters T = icp(source, target);
    float matchRatio = icp.errorMinimizer->getWeightedPointUsedRatio();
    cout << "match ratio: " <<  matchRatio << endl;

    PointCloud source_out(source);
    icp.transformations.apply(source_out, T);

    cout << endl << "------------------" << endl;

    // START demo 1
    // Test for retrieving Haussdorff distance (with outliers). We generate new matching module
    // specifically for this purpose.
    //
    // INPUTS:
    // target: point cloud used as reference
    // source_out: aligned point cloud (using the transformation outputted by icp)
    // icp: icp object used to aligned the point clouds

    // Structure to hold future match results
    PM::Matches matches;

    Parametrizable::Parameters params;
    params["knn"] =  toParam(1); // for Hausdorff distance, we only need the first closest point
    params["epsilon"] =  toParam(0);

    PM::Matcher* matcherHausdorff = PM::get().MatcherRegistrar.create("KDTreeMatcher", params);

    // max. distance from reading to reference
    matcherHausdorff->init(target);
    matches = matcherHausdorff->findClosests(source_out);
    float maxDist1 = matches.getDistsQuantile(1.0);
    float maxDistRobust1 = matches.getDistsQuantile(0.85);

    // max. distance from reference to reading
    matcherHausdorff->init(source_out);
    matches = matcherHausdorff->findClosests(target);
    float maxDist2 = matches.getDistsQuantile(1.0);
    float maxDistRobust2 = matches.getDistsQuantile(0.85);

    float haussdorffDist = std::max(maxDist1, maxDist2);
    float haussdorffQuantileDist = std::max(maxDistRobust1, maxDistRobust2);

    cout << "Haussdorff distance: " << std::sqrt(haussdorffDist) << " m" << endl;
    cout << "Haussdorff quantile distance: " << std::sqrt(haussdorffQuantileDist) <<  " m" << endl;

    // START demo 2
    // Test for retrieving paired point mean distance without outliers. We reuse the same module used for
    // the icp object.
    //
    // INPUTS:
    // target: point cloud used as reference
    // source_out: aligned point cloud (using the transformation outputted by icp)
    // icp: icp object used to aligned the point clouds

    // initiate the matching with unfiltered point cloud
    icp.matcher->init(target);

    // extract closest points
    matches = icp.matcher->findClosests(source_out);

    // weight paired points
    const PM::OutlierWeights outlierWeights = icp.outlierFilters.compute(source_out, target, matches);

    // generate tuples of matched points and remove pairs with zero weight
    const PM::ErrorMinimizer::ErrorElements matchedPoints( source_out, target, outlierWeights, matches);

    // extract relevant information for convenience
    const int dim = matchedPoints.reading.getEuclideanDim();
    const int nbMatchedPoints = matchedPoints.reading.getNbPoints();
    const PM::Matrix matchedRead = matchedPoints.reading.features.topRows(dim);
    const PM::Matrix matchedRef = matchedPoints.reference.features.topRows(dim);

    // compute mean distance
    const PM::Matrix dist = (matchedRead - matchedRef).colwise().norm(); // replace that by squaredNorm() to save computation time
    const float meanDist = dist.sum()/nbMatchedPoints;
    cout << "Robust mean distance: " << meanDist << " m" << endl;

    // END demo

    cout << "------------------" << endl << endl;

    source_out.save("source_out.vtk");
    if(isTransfoSaved)
    {
        ofstream transfoFile;
        string initFileName = "init_transfo.txt";
        string icpFileName = "icp_transfo.txt";
        string completeFileName = "complete_transfo.txt";

        transfoFile.open(initFileName.c_str());
        if(transfoFile.is_open())
        {
            transfoFile << initTransfo << endl;
            transfoFile.close();
        }
        else
        {
            cout << "Unable to write the initial transformation file\n" << endl;
        }

        transfoFile.open(icpFileName.c_str());
        if(transfoFile.is_open())
        {
            transfoFile << T << endl;
            transfoFile.close();
        }
        else
        {
            cout << "Unable to write the ICP transformation file\n" << endl;
        }

        transfoFile.open(completeFileName.c_str());
        if(transfoFile.is_open())
        {
            //Conversion from dynamic to static to avoid bug
            Eigen::Matrix<float, 4, 4> initTransfo1=initTransfo;
            Eigen::Matrix<float, 4, 4> T1=T;
            transfoFile << T1*initTransfo1 << endl;
            transfoFile.close();
        }
        else
        {
            cout << "Unable to write the complete transformation file\n" << endl;
        }
    }
    else
    {
        Eigen::Matrix<float, 4, 4> initTransfo1=initTransfo;
        Eigen::Matrix<float, 4, 4> T1=T;
        cout << "ICP transformation:" << endl << T << endl;
        cout << "Complete transformation:" << endl << T1*initTransfo1 << endl;
    }

    return 0;
}

// Make sure that the command arguments make sense
int validateArgs(const int argc, const char *argv[],
                 bool& isTransfoSaved,
                 string& configFile,
                 string& initTranslation, string& initRotation)
{
    if (argc == 1)
    {
        return 1;
    }
    else if (argc == 2)
    {
            return 2;
    }

    const int endOpt(argc - 2);
    for (int i = 1; i < endOpt; i += 2)
    {
        const string opt(argv[i]);
        if (i + 1 > endOpt)
        {
            cerr << "Missing value for option " << opt << ", usage:"; usage(argv); exit(1);
        }
        if (opt == "--isTransfoSaved")
        {
            if (strcmp(argv[i+1], "1") == 0 || strcmp(argv[i+1], "true") == 0) {
                isTransfoSaved = true;
            }
            else if (strcmp(argv[i+1], "0") == 0
                     || strcmp(argv[i+1],"false") == 0) {
                isTransfoSaved = false;
            }
            else
            {
                cerr << "Invalid value for parameter isTransfoSaved." << endl
                     << "Value must be true or false or 1 or 0." << endl
                     << "Default value will be used." << endl;
            }
        }
        else if (opt == "--config")
        {
            configFile = argv[i+1];
        }
        else if (opt == "--initTranslation")
        {
            initTranslation = argv[i+1];
        }
        else if (opt == "--initRotation")
        {
            initRotation = argv[i+1];
        }
        else
        {
            cerr << "Unknown option " << opt << ", usage:"; usage(argv); exit(1);
        }
    }
    return 0;
}

//---------------------------------------------------------------------------------------------------

PM::TransformationParameters parseTranslation(string& translation, const int cloudDimension)
{
    PM::TransformationParameters parsedTranslation;
    parsedTranslation = PM::TransformationParameters::Identity(
                cloudDimension+1,cloudDimension+1);

    translation.erase(std::remove(translation.begin(), translation.end(), '['),
                      translation.end());
    translation.erase(std::remove(translation.begin(), translation.end(), ']'),
                      translation.end());
    std::replace( translation.begin(), translation.end(), ',', ' ');
    std::replace( translation.begin(), translation.end(), ';', ' ');

    float translationValues[3] = {0};
    stringstream translationStringStream(translation);
    for( int i = 0; i < cloudDimension; i++)
    {
        if(!(translationStringStream >> translationValues[i]))
        {
            cerr << "An error occured while trying to parse the initial "
                 << "translation." << endl
                 << "No initial translation will be used" << endl;
            return parsedTranslation;
        }
    }
    float extraOutput = 0;
    if((translationStringStream >> extraOutput))
  {
        cerr << "Wrong initial translation size" << endl
             << "No initial translation will be used" << endl;
        return parsedTranslation;
    }

    for( int i = 0; i < cloudDimension; i++)
    {
        parsedTranslation(i,cloudDimension) = translationValues[i];
    }

    return parsedTranslation;
}

//---------------------------------------------------------------------------------------------------


PM::TransformationParameters parseRotation(string &rotation,const int cloudDimension)
{
    PM::TransformationParameters parsedRotation;
    parsedRotation = PM::TransformationParameters::Identity(cloudDimension+1,cloudDimension+1);
    rotation.erase(std::remove(rotation.begin(), rotation.end(), '['),rotation.end());
    rotation.erase(std::remove(rotation.begin(), rotation.end(), ']'),rotation.end());
    std::replace( rotation.begin(), rotation.end(), ',', ' ');
    std::replace( rotation.begin(), rotation.end(), ';', ' ');

    float rotationMatrix[9] = {0};
    stringstream rotationStringStream(rotation);
    for( int i = 0; i < cloudDimension*cloudDimension; i++) {
        if(!(rotationStringStream >> rotationMatrix[i])) {
            cerr << "An error occured while trying to parse the initial "
                 << "rotation." << endl
                 << "No initial rotation will be used" << endl;
            return parsedRotation;
        }
    }
    float extraOutput = 0;
    if((rotationStringStream >> extraOutput)) {
        cerr << "Wrong initial rotation size" << endl
             << "No initial rotation will be used" << endl;
        return parsedRotation;
    }

    for( int i = 0; i < cloudDimension*cloudDimension; i++) {
        parsedRotation(i/cloudDimension,i%cloudDimension) = rotationMatrix[i];
    }

    return parsedRotation;
}

// Dump command-line help
void usage(const char *argv[])
{
    cerr << " usage : " << argv[0] << " [OPTIONS] reference.csv reading.csv" << endl;
    cerr << endl;
    cerr << "OPTIONS can be a combination of:" << endl;
    cerr << "--config YAML_CONFIG_FILE  Load the config from a YAML file (default: default parameters)" << endl;
    cerr << "--initTranslation [x,y,z]  Add an initial 3D translation before applying ICP (default: 0,0,0)" << endl;
    cerr << "--initTranslation [x,y]    Add an initial 2D translation before applying ICP (default: 0,0)" << endl;
    cerr << "--initRotation [r00,r01,r02,r10,r11,r12,r20,r21,r22]" << endl;
    cerr << "--isTransfoSaved BOOL      Save transformation matrix in three different files:" << endl;
    cerr << "                             - BASEFILENAME_inti_transfo.txt" << endl;
    cerr << "                             - BASEFILENAME_icp_transfo.txt" << endl;
    cerr << "                             - BASEFILENAME_complete_transfo.txt" << endl;
    cerr << "                           (default: false)" << endl;
    cerr << endl;
}

