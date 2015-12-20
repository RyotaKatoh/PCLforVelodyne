#include <iostream>
#include <SimpleHDLViewer.h>

int main (int argc, char ** argv)
{
    std::string hdlCalibration, pcapFile;
    
    //parse_argument (argc, argv, "-calibrationFile", hdlCalibration);
    //parse_argument (argc, argv, "-pcapFile", pcapFile);
    pcapFile = "note1.pcap";
    
    
    pcl::HDLGrabber grabber (hdlCalibration, pcapFile);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler ("z");
    
    
    SimpleHDLViewer v(grabber, color_handler);
    v.run ();
    return (0);
}
