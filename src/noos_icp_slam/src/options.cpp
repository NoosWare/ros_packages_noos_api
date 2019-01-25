#include "options.hpp"

const std::vector<std::string> options::options_ = {"platform",
													"icp",
                                                    "loaded",
                                                    "help"};

const std::vector<std::string> options::sections_ = {"platform.address",
                                                      "platform.port",
                                                      "platform.user",
                                                      "platform.pass"};

const std::string default_file = "config/platform.ini";
   
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
        ("icp, i", boost::program_options::value<std::string>(), "icp configuration file");
    boost::program_options::options_description visible("-- ROS Noos ICP SLAM options --");
    visible.add_options()("help,h", "see help");
    visible.add(config_opt);

    return visible;
}

noos::cloud::platform options::read() 
{
    noos::cloud::platform node;
	std::string config_file;
	for (const auto key : options_) {
        if (vm.count(key) && key == "platform") {
            config_file = vm[key].as<std::string>(); 
            check_files()(config_file);
        }
		else if (vm.count(key) && key == "icp") {
			icp_data_.config_file = vm[key].as<std::string>();
			check_files()(icp_file);
		}
    }
    read_file(node);
    if (icp_data_.config_file.empty()) {
        icp_data_.loaded = true;
    }
    return node;
}

void options::read_file(noos::cloud::platform & node)
{
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(args.mario_file, pt);

    node.address = pt.get(sections_.at(0), "demo.noos.cloud");
    node.port = pt.get(sections_.at(1), "9001");
    node.user = pt.get(sections_.at(2));
    node.token = pt.get(sections_.at(3));

}
