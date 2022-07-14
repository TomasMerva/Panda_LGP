#include <gnuplot_module/gnuplot_module.h>


GnuplotModule::GnuplotModule(/* args */)
{
}

GnuplotModule::~GnuplotModule()
{
}



void GnuplotModule::PlotData(const std::vector<double> x_data, const std::vector<double> y_data, const std::string label)
{
    std::vector<std::pair<double, double>> plot_vector(x_data.size());
    for (size_t i=0; i<plot_vector.size(); i++)
    {
      plot_vector[i] = std::make_pair(x_data[i], y_data[i]);
    }

    Gnuplot gp;
    gp << "set style line " << _settings.line <<
          " linecolor rgb '#0060ad' \
            linetype " << _settings.linetype << " linewidth " << _settings.linewidth <<
          " pointtype " << _settings.pointtype << " pointsize " << _settings.pointsize <<
          "\n";

    gp << "plot" << gp.file1d(plot_vector) << "with linespoints linestyle 1 title '" << label << "'\n";
}

void GnuplotModule::SubplotData(const std::vector<double> x_data, const std::vector<std::vector<double>> y_data, const std::vector<std::string> labels)
{
    std::vector< std::vector<std::pair<double, double>> > plot_vector;
    for (size_t data=0; data<y_data.size(); data++)
    {
        std::vector<std::pair<double, double>> temp;
        for (size_t i=0; i< y_data[data].size(); i++)
        {
            auto temp_pair = std::make_pair(x_data[i], y_data[data][i]);
            temp.push_back(temp_pair);
        }
        plot_vector.push_back(temp);
    }
    int dim = y_data.size() / 2;


    Gnuplot gp;
    gp << "set multiplot layout " << dim << "," << dim << "\n";
    gp << "set style line " << _settings.line <<
          " linecolor rgb '#0060ad' \
            linetype " << _settings.linetype << " linewidth " << _settings.linewidth <<
          " pointtype " << _settings.pointtype << " pointsize " << _settings.pointsize <<
          "\n";
    
    for (int i=0; i<y_data.size(); i++)
    {
        // gp << "set yrange[" << _y_limits[i].first << ":" << _y_limits[i].second << "]\n";
        gp << "plot" << gp.file1d(plot_vector[i]) << "with linespoints linestyle 1 title '" << labels[i] << "'\n";
    }


}


void GnuplotModule::Subplot_Yranges(std::vector<std::pair<double, double>> range)
{
  _y_limits = range;
}
