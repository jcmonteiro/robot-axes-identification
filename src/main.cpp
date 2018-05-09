#include <iostream>
#include <Eigen/Dense>

#include "DataParser.hpp"
#include "Identification.hpp"

using namespace axes_ident;

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "[Error] Only one argument containing the file name is expected!" << std::endl;
        return 1;
    }

    DataParser parser;
    parser.setFilter( {3,4,5} );
    parser.setDelimiter('\t');
    if (!parser.readFile(argv[1]))
        return 1;

    Identification ident(parser.getNJoints());
    ident.setData(parser.getData());
    ident.identifyAxes();

    return 0;
}
