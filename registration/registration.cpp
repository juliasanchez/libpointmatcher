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

int validateArgs(const int argc, const char *argv[],bool& isTransfoSaved,string& initTranslation, string& initRotation);
PM::TransformationParameters parseTranslation(string& translation);
PM::TransformationParameters parseRotation(string& rotation);
void usage(const char *argv[]);

int main(int argc, const char *argv[])
{
    bool isTransfoSaved = false;
    string initTranslation("0,0,0");
    string initRotation("1,0,0;0,1,0;0,0,1");
    const int ret = validateArgs(argc, argv, isTransfoSaved, initTranslation, initRotation);

    const char *src_file(argv[argc-2]);
    const char *tgt_file(argv[argc-1]);

    // Load point clouds
    setLogger(PM::get().LoggerRegistrar.create("FileLogger"));
    PointCloud source(PointCloud::load(src_file));
    PointCloud target(PointCloud::load(tgt_file));

//---------------------------------------STEP 1 : FILTERS----------------------------------------------------------------------

    ifstream file("configuration/filters_configuration.yaml");
    PM::DataPointsFilters filter(file);
    filter.apply(source);
    filter.apply(target);

//---------------------------------------STEP 2 : INITIAL TRANSFORMATION-------------------------------------------------------

//initialisation de la transformation
    PM::TransformationParameters translation1 = parseTranslation(initTranslation);
    Eigen::Matrix<float, 4, 4> translation=translation1;//problème avec la multiplication dynamique avec eigen: Je passe par un intermédiaire statique
    PM::TransformationParameters rotation1 = parseRotation(initRotation);
    Eigen::Matrix<float, 4, 4> rotation=rotation1;//problème avec la multiplication dynamique avec eigen: Je passe par un intermédiaire statique
    PM::TransformationParameters initTransfo = translation*rotation;

    PM::Transformation* rigidTrans;
    rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

    if (!rigidTrans->checkParameters(initTransfo))
    {
        cerr << endl<< "Initial transformation is not rigid, identiy will be used"<< endl;
        initTransfo.setIdentity(4,4);
    }

    const PointCloud initializedSource = rigidTrans->compute(source, initTransfo);

//---------------------------------------STEP 3 : ICP--------------------------------------------------------------------------
    PM::ICP icp;

    ifstream ifs("configuration/icp_configuration.yaml");
    icp.loadFromYaml(ifs);

    int cloudDimension = target.getEuclideanDim();

    PM::TransformationParameters T = icp(source, target);

    PointCloud source_out(source);
    icp.transformations.apply(source_out, T);

    source_out.save("source_out.csv");

////---------------------------------------STEP 4 : EVALUATION---------------------------------------------------------------------

    cout << endl << "--------EVALUATION----------" << endl;

    // EVALUATION paired point mean distance without outliers.

    // initiate the matching with unfiltered point cloud
    icp.matcher->init(target);

    // extract closest points
    PM::Matches matches;
    matches = icp.matcher->findClosests(source_out);

    // weight paired points
    const PM::OutlierWeights outlierWeights = icp.outlierFilters.compute(source_out, target, matches);

    // generate tuples of matched points and remove pairs with zero weight
    const PM::ErrorMinimizer::ErrorElements matchedPoints( source_out, target, outlierWeights, matches);

    // extract relevant information for convenience
    const int dim = 3;
    const int nbMatchedPoints = matchedPoints.reading.getNbPoints();
    const PM::Matrix matchedRead = matchedPoints.reading.features.topRows(dim); // [x1,x2,x3...;y1,y2,y3...;z1,z2,z3;...]
    const PM::Matrix matchedRef = matchedPoints.reference.features.topRows(dim);



    ofstream matchesFile1;
    string file_name1 = "matches1.txt";

    matchesFile1.open(file_name1.c_str());
    matchesFile1 << matchedRead.transpose() << endl;
    matchesFile1.close();

    ofstream matchesFile2;
    string file_name2 = "matches2.txt";

    matchesFile2.open(file_name2.c_str());
    matchesFile2 << matchedRef.transpose() << endl;
    matchesFile2.close();


    // compute mean distance
    const PM::Matrix dist = (matchedRead - matchedRef).colwise().norm(); // compute the norm for each column vector =euclidian distance
    const float meanDist = dist.sum()/nbMatchedPoints;
    cout << "mean distance without outliers: " << meanDist << " m" << endl;

//    cout << "------------------" << endl << endl;

////---------------------------------------STEP 5 : SAVE RESULTS---------------------------------------------------------------------

//    if(isTransfoSaved)
//    {
//        ofstream transfoFile;
//        string initFileName = "init_transfo.txt";
//        string icpFileName = "icp_transfo.txt";
//        string completeFileName = "complete_transfo.txt";

//        transfoFile.open(initFileName.c_str());
//        if(transfoFile.is_open())
//        {
//            transfoFile << initTransfo << endl;
//            transfoFile.close();
//        }
//        else
//        {
//            cout << "Unable to write the initial transformation file\n" << endl;
//        }

//        transfoFile.open(icpFileName.c_str());
//        if(transfoFile.is_open())
//        {
//            transfoFile << T << endl;
//            transfoFile.close();
//        }
//        else
//        {
//            cout << "Unable to write the ICP transformation file\n" << endl;
//        }

//        transfoFile.open(completeFileName.c_str());
//        if(transfoFile.is_open())
//        {
//            //Conversion from dynamic to static to avoid bug
//            Eigen::Matrix<float, 4, 4> initTransfo1=initTransfo;
//            Eigen::Matrix<float, 4, 4> T1=T;
//            transfoFile << T1*initTransfo1 << endl;
//            transfoFile.close();
//        }
//        else
//        {
//            cout << "Unable to write the complete transformation file\n" << endl;
//        }
//    }
//    else
//    {
//        Eigen::Matrix<float, 4, 4> initTransfo1=initTransfo;
//        Eigen::Matrix<float, 4, 4> T1=T;
//        cout << "ICP transformation:" << endl << T << endl;
//        cout << "Complete transformation:" << endl << T1*initTransfo1 << endl;
//    }

    return 0;
}

// Make sure that the command arguments make sense
int validateArgs(const int argc, const char *argv[],
                 bool& isTransfoSaved,
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

PM::TransformationParameters parseTranslation(string& translation)
{
    const int cloudDimension=3;
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


PM::TransformationParameters parseRotation(string &rotation)
{
    const int cloudDimension=3;
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
