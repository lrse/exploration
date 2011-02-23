#include <iostream>
#include <list>
#include <sstream>
#include "plotter.h"

using namespace HybNav;
using namespace std;
namespace bp = ::boost::process;

/**************************
 * Constructor/Destructor *
 **************************/

Plotter::Plotter(void) : Singleton<Plotter>(this), gnuplot_process(launch_process()) {
  *this << "set datafile nofpe_trap";
}

bp::child Plotter::launch_process(void) {
  bp::context ctx;
  ctx.stdout_behavior = bp::inherit_stream();
  ctx.stdin_behavior = bp::capture_stream();
  ctx.stderr_behavior = bp::inherit_stream();
  ctx.environment = bp::self::get_environment();
  return bp::launch(bp::find_executable_in_path("gnuplot"), list<string>(1, string("")), ctx);
}

Plotter::~Plotter(void) {
  cout << "Terminating gnuplot process..." << endl;
  *this << "quit";
  gnuplot_process.wait();
  //gnuplot_process.terminate(true);
}

/**************************
 *     Public Methods     *
 **************************/

void Plotter::operator<<(const char* cmd) {
  *this << string(cmd);
}

void Plotter::operator<<(const string& cmd) {
  //cout << "cmd: " << cmd << endl;
  gnuplot_process.get_stdin() << cmd << endl;
}

Plotter& Plotter::plot(std::list<Plot> plot_list) {
  // construct final plot command
  string command = "plot ";
  for (std::list<Plot>::iterator it = plot_list.begin(); it != plot_list.end(); ++it) {
    if (it != plot_list.begin()) command += ", ";
    command += it->command;
  }

  // send command
  *this << command;
  
  // send all matrix data
  for (std::list<Plot>::iterator it = plot_list.begin(); it != plot_list.end(); ++it)
    *this << it->matrix;

  gnuplot_process.get_stdin().flush();
  return *this;
}

Plotter& Plotter::plot(const Plot& p) {
  list<Plot> pl;
  pl.push_back(p);
  plot(pl);
  return *this;
}

/**************************
 *    Private Methods     *
 **************************/

void Plotter::operator<<(const gsl::matrix& matrix_data)
{
  size_t bytes = matrix_data.size1() * matrix_data.size2() * sizeof(double);
  cout << "bytes: " << bytes << " size1: " << matrix_data.size1() << " size2: " << matrix_data.size2() << endl;
  gnuplot_process.get_stdin().write((const char*)(matrix_data.gslobj()->data), bytes);
}

/**************************
 * Constructor/Destructor *
 **************************/

Plot::Plot(const gsl::matrix& _matrix, const std::string& plot_with, const std::string& extra_commands) : matrix(_matrix) {
  ostringstream ostr;
  if (plot_with == "image")
    ostr << "'-' binary array=(" << matrix.size1() << "," << matrix.size2() << ") format='%double' " << extra_commands << " with image";
  else {
    ostr << "'-' binary record=(" << matrix.size1() << ") format=\"";
    for (uint i = 0; i < matrix.size2(); i++) ostr << "%double";
    ostr << "\" " << extra_commands << " with " << plot_with;
  }
  command = ostr.str();
}