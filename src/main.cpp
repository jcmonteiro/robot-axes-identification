#include <iostream>
#include <Eigen/Dense>

#include "DataParser.hpp"

using namespace axes_ident;

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "[Error] Only one argument containing the file name is expected!" << std::endl;
        return 1;
    }

    DataParser::Data data;

    DataParser parser;
    parser.setFilter( {3,4,5} );
    if (!parser.readFile(argv[1], data))
        return 1;
    parser.appendMovingJointIndex(data);

    std::cout << data << std::endl;

    return 0;
}
