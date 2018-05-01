#pragma once

#include <fstream>
#include <functional>
#include <vector>
#include <Eigen/Dense>

namespace axes_ident
{

class DataParser
{
public:
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Data;

private:
    char delim;
    unsigned int header_size;
    std::vector<unsigned int> filter;

    void jumpHeader(std::ifstream &file);
    void processLine(std::string &line, std::function<void (std::string)> callback);

public:
    DataParser();

    bool readFile(std::string fname, Data &data);
    void appendMovingJointIndex(Data &data);

    inline void setDelimiter(char val)
    {
        delim = val;
    }

    inline void setHeaderSize(int val)
    {
        header_size = val;
    }

    inline void setFilter(const std::vector<unsigned int> val)
    {
        filter = val;
    }
};

}