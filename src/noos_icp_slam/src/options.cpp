#include "options.hpp"

const std::vector<std::string> options::options_ = {"platform",
													"icp",
                                                    "loaded",
                                                    "robot_name",
                                                    "scan_topic",
                                                    "map_name"
                                                    "help"};

const std::vector<std::string> options::sections_ = {"platform.address",
                                                      "platform.port",
                                                      "platform.user",
                                                      "platform.pass"};

const std::string options::default_file = "config/platform.ini";
   
void check_files::operator()(const std::string filename)
{
    std::ifstream f(filename.c_str());
    if(!f.good()) {
        throw std::runtime_error("Options error: Configuration file " + filename + " not found");
    }
}

options::options(int & argc, char * argv[])
{
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv)
                                 .options(options::description())
                                 .style(boost::program_options::command_line_style::unix_style
                                      | boost::program_options::command_line_style::allow_short)
                                 .run(), vm);
    boost::program_options::notify(vm);
    check_number_args(argc, vm.count("help"));
    if (vm.count("help")) {
        std::cout << options::description() << std::endl;
    }

}

boost::program_options::options_description options::description() const
{
    boost::program_options::options_description config_opt("Mario configuration data:");
    config_opt.add_options()
        ("platform, p", boost::program_options::value<std::string>()->default_value("config/platform.ini"), "platform data file")
        ("loaded, l", boost::program_options::value<bool>()->default_value(false), "Boolean to indicate if the icp file has been loaded previously")
        ("robot_name, r", boost::program_options::value<std::string>()->default_value("robot0"), "Name of the robot who is doing this SLAM")
        ("scan_topic, s", boost::program_options::value<std::string>()->default_value("/scan"), "Name of the topic which reads Laser data. /scan by default.")
        ("map_name, m", boost::program_options::value<std::string>()->default_value("icp"), "Name of the map which is going to be used/created in the cloud. icp by default.")
        ("icp, i", boost::program_options::value<std::string>()->default_value("config/icp.ini"), "icp configuration file");
    boost::program_options::options_description visible("-- ROS Noos ICP SLAM options --");
    visible.add_options()("help,h", "see help");
    visible.add(config_opt);

    return visible;
}

noos::cloud::platform options::read() 
{
    noos::cloud::platform node;
	std::string config_file = default_file;
	for (const auto key : options_) {
        if (vm.count(key) && key == "platform") {
            config_file = vm[key].as<std::string>(); 
            check_files()(config_file);
        }
		else if (vm.count(key) && key == "icp") {
			icp_data_.config_file = vm[key].as<std::string>();
			check_files()(icp_data_.config_file);
		}
		else if (vm.count(key) && key == "loaded") {
            icp_data_.loaded = vm[key].as<bool>();
        }
		else if (vm.count(key) && key == "robot_name") {
			icp_data_.robot_name = vm[key].as<std::string>();
		}
		else if (vm.count(key) && key == "scan_topic") {
			icp_data_.topic = vm[key].as<std::string>();
		}
        else if (vm.count(key) && key == "map_name") {
			icp_data_.map_name = vm[key].as<std::string>();
		}
    }
    read_file(node, config_file);
    return node;
}

icp_args options::get_icp_data()
{
    return icp_data_;
}

void options::read_file(noos::cloud::platform & node,
                        std::string config_file)
{
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_file, pt);

    node.address = pt.get(sections_.at(0), "demo.noos.cloud");
    node.port = pt.get(sections_.at(1), "9001");
    node.user = pt.get<std::string>(sections_.at(2));
    node.token = pt.get<std::string>(sections_.at(3));
}

void options::check_number_args(const int argc, bool help)
{
    if (argc == 1 || (argc == 2 && help)) {
        std::cout << "WARNING: No configuration arguments added. Default configuration is going to be loaded\r\n" << std::endl;
    }
}
