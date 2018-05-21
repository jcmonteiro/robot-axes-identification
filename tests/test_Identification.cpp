#define BOOST_TEST_MODULE identification test
#include <boost/test/unit_test.hpp>

#include <Identification.hpp>

using namespace axes_ident;

bool compareMatrices(const Eigen::MatrixXd & m1, const Eigen::MatrixXd & m2, double tol)
{
    bool ret = (m1 - m2).array().abs().maxCoeff() <= tol;
    if (!ret) {
        std::cout << m1 << std::endl;
        std::cout << m2 << std::endl;
        std::cout << (m1 - m2).array().abs().maxCoeff() << std::endl;
    }
    return ret;
}

void testFile(const std::string &file, DataParser &parser, const Eigen::MatrixXd & answer_1, const Eigen::MatrixXd & answer_2)
{
    std::cout << "Testing data from " + file << std::endl;

    BOOST_REQUIRE_MESSAGE(parser.readFile(file),
        "File " + file + " not found... Some tests might have been skipped!"
    );

    const std::vector<DataParser::Data> &data = parser.getDataByJoint();
    for (auto iter = data.begin(); iter < data.end(); ++iter)
    {
        std::cout << *iter << std::endl;
        std::cout << "--" << std::endl;
    }

    Identification ident(parser.getNJoints());
    ident.setData(parser);
    auto axis_1 = ident.identifyAxes(false);
    auto axis_2 = ident.identifyAxes(true);

    double tol_zero = 5e-5;
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
    matlab_answer_1 << 0.757338, 0.872365, 0.671012, 0.356584, 0.0813102,
                       -0.14607, 0.467005, 0.686292,   0.3677,  0.727377,
                      -0.636477, 0.144518, 0.280619, 0.858862,  0.681404;
    //
    matlab_answer_2 << 0.756059, 0.873364,  0.67224, 0.353429, 0.0783598,
                      -0.145017, 0.464509, 0.684809, 0.367732,  0.727194,
                      -0.638236, 0.146514,   0.2813, 0.860152,  0.681945;
    //
    testFile("../tests/random_data.txt", parser, matlab_answer_1, matlab_answer_2);
}