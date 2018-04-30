#include "DataParser.hpp"

#include <iostream>
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
    for (auto iter_char = line.begin() ; iter_char < line.end() ; ++iter_char)
    {
        if (*iter_char != ' ' && *iter_char != delim)
        {
            str_number += *iter_char;
        }
        else if (!str_number.empty())
        {
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

bool DataParser::readFile(std::string fname, Data &ret)
{
    std::ifstream file(fname);

    if (!file.is_open())
        return false;

    std::string line;
    jumpHeader(file);

    int pos = file.tellg();
    std::getline(file, line);
    file.seekg(pos);

    unsigned int n_cols = 0;
    processLine(line, [&n_cols] (std::string number) {++n_cols;} );

    unsigned int total_rows = 100;
    ret.resize(total_rows, n_cols);

    unsigned int k = 0, n_rows = 0;
    while (std::getline(file, line))
    {
        if (++n_rows > total_rows)
        {
            total_rows *= 2;
            ret.conservativeResize(total_rows, n_cols);
        }
        processLine(line, [&ret, &k] (std::string number)
            {
                ret(k++) = std::atof(number.c_str());
            });
    }
    ret.conservativeResize(n_rows, n_cols);

    file.close();
    return true;
}