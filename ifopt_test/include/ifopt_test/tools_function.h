#include <iostream>
#include <vector>
#include <string>
#include "gnuplot-iostream.h"


template<typename T>
std::vector<double> linspace(T start_in, T end_in, int num_in)
{

  std::vector<double> linspaced;

  double start = static_cast<double>(start_in);
  double end = static_cast<double>(end_in);
  double num = static_cast<double>(num_in);

  if (num == 0) { return linspaced; }
  if (num == 1) 
    {
      linspaced.push_back(start);
      return linspaced;
    }

  double delta = (end - start) / (num - 1);

  for(int i=0; i < num-1; ++i)
    {
      linspaced.push_back(start + delta * i);
    }
  linspaced.push_back(end); // I want to ensure that start and end
                            // are exactly the same as the input
  return linspaced;
}

template<typename T>
void PlotData(const std::vector<T> x_data, const std::vector<T> y_data, const std::string label)
{
    std::vector<std::pair<T, T>> plot_vector(x_data.size());
    for (size_t i=0; i<plot_vector.size(); i++)
    {
      plot_vector[i] = std::make_pair(x_data[i], y_data[i]);
    }

    Gnuplot gp;
    // gp << "set multiplot layout 2,2\n";
    gp << "set style line 1 \
      linecolor rgb '#0060ad' \
      linetype 1 linewidth 2 \
      pointtype 7 pointsize 1.5\n";
    gp << "plot" << gp.file1d(plot_vector) << "with linespoints linestyle 1 title '" << label << "'\n";

}