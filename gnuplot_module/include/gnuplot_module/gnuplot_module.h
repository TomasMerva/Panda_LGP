#pragma once

#include "gnuplot-iostream.h"
#include <vector>

class GnuplotModule
{
    public:
        GnuplotModule(/* args */);
        ~GnuplotModule();

        void SetYrange(std::pair<double,double> range);
        void PlotData(const std::vector<double> x_data, const std::vector<double> y_data, const std::string label);
        
        void Subplot_Yranges(std::vector<std::pair<double, double>> range);
        void SubplotData(const std::vector<double> x_data, const std::vector<std::vector<double>> y_data, const std::vector<std::string> labels);
        
    private:
        struct GnuplotSettings
        {
            uint line = 1;
            uint linetype = 1;
            uint linewidth = 2;
            uint pointtype = 7;
            uint pointsize = 1.5;
        }_settings;
        std::vector<std::pair<double, double>> _y_limits;
};  










