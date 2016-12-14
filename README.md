# libpointmatcher
test of the library libpointmatcher to register pointclouds

Examples of execution for icp_test
-simplest form:
 ./test_icp ../../data_test/Hokuyo_0.csv ../../data_test/Hokuyo_1.csv
 -with all possible parameters
 ./test_icp --config icp_configuration.yaml --initTranslation [x,y,z] --initTranslation [x,y] --initRotation [r00,r01,r02,r10,r11,r12,r20,r21,r22] --isTransfoSaved 1 
 
 isTransfoSaved = 1 save transformations as: 
    -init_transfo.txt
    -icp_transfo.txt
    -complete_transfo.txt
