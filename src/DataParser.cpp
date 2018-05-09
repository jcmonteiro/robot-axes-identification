#include "DataParser.hpp"

#include <iostream>
#include <iterator>
#include <vector>

using namespace axes_ident;

DataParser::DataParser() :
    delim(' '), header_size(0), n_joints(0),
    ok_data(false), tol_max_stall_movement(0.0002)
{
}

void DataParser::_jumpHeader(std::ifstream &file)
{
    file.seekg(0);
    std::string line;
    for (int k = 0; k < header_size; ++k)
        std::getline(file, line);
    return;
}

void DataParser::_processLine(std::string &line, std::function<void (std::string)> callback)
{
    std::string str_number = "";
    unsigned int index_max = 0;
    for (auto iter_char = line.begin() ; iter_char < line.end() ; ++iter_char)
    {
        if (*iter_char != ' ' && *iter_char != delim)
        {
            str_number += *iter_char;
        }
        else if (!str_number.empty())
        {
            if (std::find(filter.begin(), filter.end(), index_max++) != filter.end())
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

void DataParser::_appendMovingJointIndex()
{
    data.conservativeResize(data.rows(), data.cols() + 1);
    n_joints = data.cols() - 4;
    auto joints = data.block(0, 0, data.rows(), n_joints);
    
    unsigned int ind_last = data.cols() - 1;
    data(0, ind_last) = DataParser::INDEX_INVALID;
    Eigen::ArrayXd last_row = joints.row(0).array(), row, row_diff;
    Eigen::ArrayXd::Index index_max, index_stall_max;
    for (unsigned int k = 1; k < data.rows(); ++k)
    {
        row = joints.row(k).array();
        row_diff = Eigen::abs(row - last_row);
        row_diff.maxCoeff(&index_max);
        row_diff(index_max) = 0;
        row_diff.maxCoeff(&index_stall_max);
        data(k, ind_last) = (row_diff(index_stall_max) > tol_max_stall_movement) ? DataParser::INDEX_INVALID : index_max;
        last_row = row;
    }
}

bool DataParser::_validateMovingJointIndices()
{
    return data.col(data.cols() - 1).array().maxCoeff() == (n_joints - 1);
}

bool DataParser::readFile(std::string fname)
{
    this->clear();

    std::ifstream file(fname);
    if (!file.is_open())
    {
        std::cerr << "[Error] Failed to open " << fname << ". Check the file path!" << std::endl;
        return false;
    }

    std::string line;
    _jumpHeader(file);

    int file_pos = file.tellg();
    std::getline(file, line);
    file.seekg(file_pos);

    unsigned int n_cols = 0;
    _processLine(line, [&n_cols] (std::string number) {++n_cols;} );
    // If the delimiter is not set correctly, only one column will be detected,
    // since the second callback in DataParser::_processLine will be called.
    // Therefore, I am assuming that if only one column is detected, than an
    // incorrect delimiter has been passed. This should not be a problem, since
    // one column data files are not valid for this application.
    if (n_cols == 1)
    {
        std::cerr << "[Error] No columns were detected. Check if the delimiter has been chosen correctly." << std::endl;
        return false;
    }

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
        _processLine(line, [this, &k] (std::string number)
            {
                this->data(k++) = std::atof(number.c_str());
            });
        ++n_rows;
    }
    data.conservativeResize(n_rows, n_cols);

    file.close();

    this->_appendMovingJointIndex();
    ok_data = this->_validateMovingJointIndices();
    return ok_data;
}