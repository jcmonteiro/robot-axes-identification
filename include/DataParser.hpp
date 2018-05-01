#pragma once

#include <fstream>
#include <vector>
#include <Eigen/Dense>

namespace axes_ident
{

class DataParser
{
public:
    /**
     * @brief Type used to store values read from a data file.
     */
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Data;

private:
    char delim;
    unsigned int header_size;
    std::vector<unsigned int> filter;

    /**
     * @brief Jumps through the data file header lines.
     * 
     * @param file stream that describes the file.
     */
    void jumpHeader(std::ifstream &file);

    /**
     * @brief Reads a line of values and calls the callback for each number.
     * 
     * @param line the line to be read.
     * @param callback the callback that is called for each number.
     */
    void processLine(std::string &line, std::function<void (std::string)> callback);

public:
    /**
     * @brief Construct a new Data Parser object.
     */
    DataParser();

    /**
     * @brief Reads a data file and stores the result in the matrix \p data.
     * 
     * @param fname full file name.
     * @param data matrix where the data is stored.
     * @return true if the file is read successfully.
     * @return false if the file cannot be read.
     */
    bool readFile(std::string fname, Data &data);

    /**
     * @brief Appends a column indicating which joint moved from the last to the current row.
     * 
     * @param data matrix containing the experimental data.
     */
    void appendMovingJointIndex(Data &data);

    /**
     * @brief Sets the delimiter character (besides empty spaces) that separates the values in the data file.
     * 
     * @param val the delimiter.
     */
    inline void setDelimiter(char val)
    {
        delim = val;
    }

    /**
     * @brief Set the size of the header in the data file.
     * 
     * @param val the header size.
     */
    inline void setHeaderSize(int val)
    {
        header_size = val;
    }

    /**
     * @brief Chooses which columns should be filtered out from the data file.
     * 
     * @param val the columns to filter.
     */
    inline void setFilter(const std::vector<unsigned int> val)
    {
        filter = val;
    }
};

}