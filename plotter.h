#ifndef __PLOTTER_H__
#define __PLOTTER_H__

#define BOOST_FILESYSTEM_VERSION 2
#include <boost/process.hpp>
#include <gslwrap/matrix_double.h>
#include <list>
#include "singleton.h"

namespace HybNav {
  class Plot;
  
  class Plotter : public Singleton<Plotter>  {
    public:
      Plotter(void);
      ~Plotter(void);

      void operator<<(const char* cmd);
      void operator<<(const std::string& cmd);
      Plotter& plot(std::list<Plot> plot_list);
      Plotter& plot(const Plot& p);

    private:
      friend class Plot;
      void operator<<(const gsl::matrix& matrix_data);

      boost::process::child gnuplot_process;

      boost::process::child launch_process(void);
  };

  class Plot {
    public:
      Plot(const Plot& other) : matrix(other.matrix), command(other.command) { }
      Plot(const gsl::matrix& matrix, const std::string& plot_with, const std::string& extra_commands = std::string());

      const gsl::matrix& matrix;
      std::string command;
  };
}

#endif
