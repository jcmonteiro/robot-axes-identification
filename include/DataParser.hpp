#pragma once

#include <iostream>
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

    /**
     * @brief Moving joint index when the experiment is not valid.
     */
    const static int INDEX_INVALID = -1;

    /**
     * @brief Storage type.
     */
    enum Storage
    {
        MULTIPLE = (1u << 0),
        SINGLE   = (1u << 1)
    };

private:
    char delim;
    unsigned int header_size, n_joints;
    std::vector<unsigned int> filter;
    Data data;
    std::vector<Data> data_by_joint;
    bool ok_data;
    double tol_max_stall_movement;
    unsigned short mask_storage;

    /**
     * @brief Jumps through the data file header lines.
     * 
     * @param file stream that describes the file.
     */
    void _jumpHeader(std::ifstream &file);

    /**
     * @brief Reads a line of values and calls the callback for each number.
     * 
     * @param line the line to be read.
     * @param callback the callback that is called for each number.
     */
    void _processLine(std::string &line, std::function<void (std::string)> callback);

    /**
     * @brief Appends a column (to the data matrix) indicating which joint moved from the last to the current row.
     * 
     * If movement above a threshold value is detected on the ramaining joints, the experiment
     * is considered invalid and \ref DataParser.INDEX_INVALID is added in the end of the row.
     */
    void _appendMovingJointIndex();

    /**
     * @brief Validates the experimental data, checking if there are as many experiments needed to identify every joint.
     * 
     * @return true if there are as many indices as joints
     * @return false if there are less indices than joints
     */
    bool _validateMovingJointIndices() const;

    /**
     * @brief Validates the tolerance value, setting it to tol = abs(tol) if it is negative.
     * 
     * @param tol tolerance value
     */
    inline void _validateTolerance(double &tol) const
    {
        if (tol < 0)
        {
            std::cerr << "[Error] Negative tolerance value. Changing from " << tol <<
            " to " << (-tol) << '.' << std::endl;
            tol = -tol; 
        }
    }

    /**
     * @brief Configure storage based on the storage mask.
     */
    void _arrangeStorage();

    inline bool _hasStorageMask(Storage mask) const
    {
        return (this->mask_storage & mask) == mask;
    }

public:
    /**
     * @brief Construct a new Data Parser object.
     */
    DataParser();

    /**
     * @brief Maximum allowed movement on the joints that should not have moved.
     * 
     * @param tol_stall tolerance value
     */
    inline void setToleranceStall(double tol_stall)
    {
        this->_validateTolerance(tol_stall);
        tol_max_stall_movement = tol_stall;
    }

    /**
     * @brief Reads a data file and stores the results internally.
     * 
     * @param fname full file name.
     * @return true if the file is read successfully.
     * @return false if the file cannot be read.
     * @see getData
     */
    bool readFile(std::string fname);

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

    /**
     * @brief Check if the data contains any errors
     * 
     * @return true if no errors are found
     * @return false if errors are found
     */
    inline bool check() const
    {
        return ok_data;
    }

    /**
     * @brief Clear stored data.
     */
    inline void clear()
    {
        data.resize(0,0);
        n_joints = 0;
        ok_data = false;
    }

    /**
     * @brief Matrix with data read from file.
     * 
     * @return constant reference to the data matrix
     */
    inline const Data & getData() const
    {
        return data;
    }

    /**
     * @brief Get the Data By Joint object
     * 
     * @return constant reference to the vector containing data matrices for each joint
     */
    inline const std::vector<Data> & getDataByJoint() const
    {
        return data_by_joint;
    }

    /**
     * @brief Number of joints.
     */
    inline unsigned int getNJoints() const
    {
        return n_joints;
    }

    /**
     * @brief Set the Storage Mask object
     */
    inline void setStorageMask(unsigned short mask)
    {
        if ( !this->_hasStorageMask(Storage::SINGLE) &&
             !this->_hasStorageMask(Storage::MULTIPLE) )
        {
            std::cerr << "[Error] Invalid storage mask. DataParser::Storage enum for valid types." << std::endl;
            return;
        }
        mask_storage = mask;
    }
};

}