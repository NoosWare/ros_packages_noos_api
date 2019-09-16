#ifndef OPTIONS_HPP
#define OPTIONS_HPP
#include "includes.ihh"

/**
 * @struct icp_args
 * @brief arguments required for running icp
 * @date 25.01.2019
 * @version 0.1.0
 * @author Maria Ramos
 */
struct icp_args
{
    std::string config_file;
    std::string robot_name;
    std::string topic;
    std::string map_name;
    bool loaded = false;
    noos::object::pose2d<float> init_pose;
    bool update = true;
};

/**
 * @struct check_files 
 * @brief check files exist 
 * @date 25.01.2019
 * @version 0.1.0
 * @author Maria Ramos
 */
struct check_files
{
    void operator()(std::string filename);
};

/**
 * @class options
 * @brief parses the options from the command line and configuration files
 * @date 25.01.2019
 * @version 0.1.0
 * @author Maria Ramos
 */
class options
{
public:

    /// @brief constructor taking arguments
    options(int &argc, char * argv[]);
    
    /// @brief read and save arguments given
    noos::cloud::platform read();

    /// @return description of the options given
    boost::program_options::options_description description() const;

    /// @return icp arguments 
    icp_args get_icp_data();

protected:
    /// @brief read data from filter file
    void read_file(noos::cloud::platform & node,
                   std::string config_file);

    /// @brief check the number of arguments received
    void check_number_args(const int argc, bool help);

private:
    ///options given by user
    boost::program_options::variables_map vm;    
    ///the possible options
    static const std::vector<std::string> options_;
    ///general section options
    static const std::vector<std::string> sections_;
    ///configuration file
	static const std::string default_file;
    ///icp arguments
    icp_args icp_data_;
};

#endif
