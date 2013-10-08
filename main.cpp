#include <iostream>
#include <signal.h>
#include <boost/program_options.hpp>
#include "death_handler.h"
#include "exabot.h"
#include "util.h"
using namespace std;
namespace po = boost::program_options;

bool end = false;

void catch_interrupt(int sig) {
  cout << "Stopping..." << endl;
  end = true;
}

po::variables_map process_commandline(int argc, char** argv)
{
  po::options_description options_description("options");
  options_description.add_options()
    ("help,h", "display this help")
    ("cells,c", po::value<int>()->required(), "size of side of each local grid")
    ("output,o", po::value<string>()->required(), "output directory")
  ;

  po::variables_map config_vars;
  try {
    po::store(po::command_line_parser(argc, argv).options(options_description).allow_unregistered().run(), config_vars);
    //po::store(po::parse_command_line(argc, argv, options_description), config_vars);
    if (config_vars.count("help")) { cerr << options_description << endl; exit(1); }

    po::notify(config_vars);
    
    if (config_vars["cells"].as<int>() < 1) throw std::runtime_error("number cells should be higher than 1");
  }
  catch(const std::exception& e) {
    cerr << options_description << endl << endl;
    cerr << "ERROR: " << e.what() << endl;
    exit(1);
  }
  return config_vars;
}

void setup_variables(const po::variables_map& vars)
{
  HybNav::OccupancyGrid::CELLS = vars["cells"].as<int>();
  HybNav::OccupancyGrid::SIZE = HybNav::OccupancyGrid::CELL_SIZE * HybNav::OccupancyGrid::CELLS;
  HybNav::OUTPUT_DIRECTORY = vars["output"].as<string>();
}

int main(int argc, char** argv) {
  Debug::DeathHandler dh;
  signal(SIGINT, catch_interrupt);
  signal(SIGTERM, catch_interrupt);

  po::variables_map vars = process_commandline(argc, argv);
  setup_variables(vars);

  HybNav::ExaBot exabot;

  try {
    while(true) {
      exabot.update();
      usleep(0);
      if (end) break;
    }
  }
  catch(const HybNav::ExplorerException& e) {
    if (e.code == HybNav::ExplorerException::END) cout << "Exploration Ended OK" << endl;
    else cout << "Exploration Exception: " << e.what() << endl;
  }

  exabot.stop();
  exabot.update_player();
  exabot.deinitialize();

  return 0;
}

