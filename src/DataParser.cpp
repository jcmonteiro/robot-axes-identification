#include "DataParser.hpp"

#include <iostream>
#include <iterator>
#include <vector>

using namespace axes_ident;

DataParser::DataParser() :
    delim('\t'), header_size(0)
{
}

void DataParser::jumpHeader(std::ifstream &file)
{
    file.seekg(0);
    std::string line;
    for (int k = 0; k < header_size; ++k)
        std::getline(file, line);
    return;
}

void DataParser::processLine(std::string &line, std::function<void (std::string)> callback)
{
    std::string str_number = "";
    unsigned int index = 0;
    for (auto iter_char = line.begin() ; iter_char < line.end() ; ++iter_char)
    {
        if (*iter_char != ' ' && *iter_char != delim)
        {
            str_number += *iter_char;
        }
        else if (!str_number.empty())
        {
            if (std::find(filter.begin(), filter.end(), index++) != filter.end())
                continue;
            callback(str_number);
            str_number = "";
        }
    }
    if (!str_number.empty())
    {
        callback(str_number);
        str_number = "";
    }
}

bool DataParser::readFile(std::string fname, Data &data)
{
    std::ifstream file(fname);

    if (!file.is_open())
    {
        std::cerr << "[Error] Failed to open " << fname << ". Check the file path!" << std::endl;
        return false;
    }

    std::string line;
    jumpHeader(file);

    int file_pos = file.tellg();
    std::getline(file, line);
    file.seekg(file_pos);

    unsigned int n_cols = 0;
    processLine(line, [&n_cols] (std::string number) {++n_cols;} );

    // stop skipping new lines to count the amount of lines
    file.unsetf(std::ios_base::skipws);

    // count the newlines
    const unsigned int total_rows = std::count(
        std::istream_iterator<char>(file),
        std::istream_iterator<char>(), 
        '\n');

    // don't know why, but the file has to be closed after running this newline counter...
    // reseting std::ios_base::skipws and calling seekg did not work
    file.close();
    file.open(fname);
    file.seekg(file_pos);

    data.resize(total_rows, n_cols);

    unsigned int k = 0, n_rows = 0;
    while (std::getline(file, line))
    {
        if (line.empty())
        {
            std::clog << "[Warn] File will not be processed any further due to an empty line" << std::endl;
            break;
        }
        processLine(line, [&data, &k] (std::string number)
            {
                data(k++) = std::atof(number.c_str());
            });
        ++n_rows;
    }
    data.conservativeResize(n_rows, n_cols);

    file.close();
    return true;
}

void DataParser::appendMovingJointIndex(Data &data)
{
    data.conservativeResize(data.rows(), data.cols() + 1);
    unsigned int n_joints = data.cols() - 4;
    auto joints = data.block(0, 0, data.rows(), n_joints);
    
    unsigned int ind_last = data.cols() - 1;
    data(0, ind_last) = -1;
    auto last_row = joints.row(0).array();
    Eigen::ArrayXd::Index index;
    for (unsigned int k = 1; k < data.rows(); ++k)
    {
        auto row = joints.row(k).array();
        Eigen::abs(row - last_row).maxCoeff(&index);
        data(k, ind_last) = index;
        last_row = row;
    }
}