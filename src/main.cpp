#include <iostream>
#include <Eigen/Dense>

#include "DataParser.hpp"

using namespace axes_ident;

int main()
{
    DataParser::Data data;

    DataParser parser;
    parser.readFile("/home/jcmonteiro/Documents/coding/axes_ident/panda.txt", data);

    std::cout << data << std::endl;

    return 0;
}
