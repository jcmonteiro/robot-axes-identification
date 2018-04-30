#pragma once

#include <fstream>
#include <functional>
#include <Eigen/Dense>

namespace axes_ident
{

class DataParser
{
public:
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Data;

private:
    char delim;
    int header_size;

    void jumpHeader(std::ifstream &file);
    void processLine(std::string &line, std::function<void (std::string)> callback);

public:
    DataParser();

    bool readFile(std::string fname, Data &ret);

    inline void setDelimiter(char val)
    {
        delim = val;
    }

    inline void setHeaderSize(int val)
    {
        header_size = val;
    }
};

}