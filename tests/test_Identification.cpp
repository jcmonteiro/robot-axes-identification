#define BOOST_TEST_MODULE identification test
#include <boost/test/unit_test.hpp>

#include <Identification.hpp>

using namespace axes_ident;

bool compareMatrices(const Eigen::MatrixXd & m1, const Eigen::MatrixXd & m2, double tol)
{
    return (m1 - m2).array().abs().maxCoeff() <= tol;
}

void testFile(const std::string &file, DataParser &parser, const Eigen::MatrixXd & answer_1, const Eigen::MatrixXd & answer_2, double tol_zero = 5e-5)
{
    std::cout << "Testing data from " + file << std::endl;

    BOOST_REQUIRE_MESSAGE(parser.readFile(file),
        "File " + file + " not found... Some tests might have been skipped!"
    );

    Identification ident(parser.getNJoints());
    ident.setData(parser);
    auto axis_1 = ident.identifyAxes(false);
    auto axis_2 = ident.identifyAxes(true);

    BOOST_REQUIRE_MESSAGE(compareMatrices(axis_1, axis_2, tol_zero),
        "Identifications starting from the right and from the left are too different!");

    BOOST_REQUIRE_MESSAGE(compareMatrices(axis_1, answer_1, tol_zero),
        "Identification starting from the right differs too much from Matlab's answer!");

    BOOST_REQUIRE_MESSAGE(compareMatrices(axis_2, answer_2, tol_zero),
        "Identification starting from the left differs too much from Matlab's answer!");

    parser.clear();
}

BOOST_AUTO_TEST_CASE( identification_test )
{
    DataParser parser;
    parser.setFilter( {3,4,5} );
    parser.setDelimiter('\t');
    parser.setStorageMask( DataParser::Storage::MULTIPLE );

    Eigen::MatrixXd matlab_answer_1(3,3);
    Eigen::MatrixXd matlab_answer_2(3,3);

    // panda.txt
    matlab_answer_1 << -0.0000290224, 0.0000099693,    0.980581,
                            0.196116,      0.98058,    0.196116,
                            0.980581,     0.196119, -1.54946e-7;
    //
    matlab_answer_2 << -2.06872e-9, 0.00000300174,      0.980581,
                          0.196115,       0.98058,      0.196117,
                          0.980581,       0.19612, 0.00000236851;
    //
    testFile("../tests/panda.txt", parser, matlab_answer_1, matlab_answer_2);

    // parrot.txt
    matlab_answer_1 << -1.02332e-8, 0.00000480051,            1.0,
                       -8.38736e-9,           1.0, -0.00000469248,
                               1.0,    2.17543e-8,  0.00000469916;
    //
    matlab_answer_2 << 2.80319e-10, -6.59774e-8,          1.0,
                        3.74344e-9,         1.0,  5.07646e-11,
                               1.0, -5.74527e-7, -9.92273e-10;
    //
    testFile("../tests/parrot.txt", parser, matlab_answer_1, matlab_answer_2);

    // random.txt
    matlab_answer_1.resize(3,5);
    matlab_answer_2.resize(3,5);
    parser.setFilter({});
    matlab_answer_1 << 0.757292, 0.872405, 0.671113, 0.356736,  0.08158,
                      -0.146086, 0.466946, 0.686207, 0.367552, 0.727297,
                      -0.636528,  0.14447, 0.280585, 0.858863, 0.681457;
    //
    matlab_answer_2 << 0.755541, 0.873041, 0.672542, 0.354553, 0.0777396,
                       -0.14505, 0.465092, 0.684404, 0.366473,  0.726473,
                      -0.638841, 0.146594, 0.281562, 0.860227,  0.682784;
    //
    testFile("../tests/random_data.txt", parser, matlab_answer_1, matlab_answer_2, 5e-2);
}