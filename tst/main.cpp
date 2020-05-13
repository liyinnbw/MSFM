  
#include "gtest/gtest.h"
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <iostream>
#include <string>

std::string programExecutionPath;
int main(int argc, char **argv) {
    srand (time(NULL));
    ::testing::InitGoogleTest(&argc, argv);

    //the following is just to get the execution path when running this test program
    //the execution path is needed to correctly find the test data
    programExecutionPath = std::string(argv[0]);
    programExecutionPath = programExecutionPath.substr(0, programExecutionPath.find_last_of("/"));
    if (programExecutionPath==""){programExecutionPath=".";}


    return RUN_ALL_TESTS();
}