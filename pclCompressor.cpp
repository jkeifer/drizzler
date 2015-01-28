#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include <stdio.h>
#include <sstream>
#include <stdlib.h>
#include <cstdio>
#include <string>
#include <bitset>
#include <dirent.h>

#include <boost/program_options.hpp>

namespace po = boost::program_options;


class PointCloudCompressor {
    public:
        // constructors
        PointCloudCompressor(std::string str);  // just a file path
        PointCloudCompressor(std::string str, pcl::io::compression_Profiles_e compprof);  // file path and comp profile
        PointCloudCompressor(std::string str, bool stats);  // file path and show statistics
        PointCloudCompressor(std::string str, pcl::io::compression_Profiles_e compprof, bool stats);  // all of the above

        //destructor
        ~PointCloudCompressor() {
            fclose(outfilestream);
            delete (PointCloudEncoder);
        };

        // member functions
        std::string getFilepath(void);
        void compressDirectory(const std::string directorypath);
        void compressFrame(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);

    private:
        const bool showStatistics = false;
        // for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
        const pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
        pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> * PointCloudEncoder;
        int framecount = 0;
        const std::string filepath;
        FILE * outfilestream;
};


// CONSTRUCTORS
// just file path
PointCloudCompressor::PointCloudCompressor(std::string str):
    filepath(str)
    {
    outfilestream = fopen(filepath.c_str(), "a+b");
    PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> (compressionProfile, showStatistics);
}


// file path and comp profile
PointCloudCompressor::PointCloudCompressor(std::string str, pcl::io::compression_Profiles_e compprof):
    filepath(str),
    compressionProfile(compprof)
    {
    outfilestream = fopen(filepath.c_str(), "a+b");
    PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> (compressionProfile, showStatistics);
}


// file path and stats
PointCloudCompressor::PointCloudCompressor(std::string str, bool stats):
    filepath(str),
    showStatistics(stats)
    {
    outfilestream = fopen(filepath.c_str(), "a+b");
    PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> (compressionProfile, showStatistics);
}


// all args
PointCloudCompressor::PointCloudCompressor(std::string str, pcl::io::compression_Profiles_e compprof, bool stats):
    filepath(str),
    compressionProfile(compprof),
    showStatistics(stats)
    {
    outfilestream = fopen(filepath.c_str(), "a+b");
    PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> (compressionProfile, showStatistics);
}


// ACCESSOR METHODS
std::string PointCloudCompressor::getFilepath(void) {
    return filepath;
}


// METHODS
void PointCloudCompressor::compressDirectory(const std::string directorypath) {
    struct dirent *dp;
    DIR * dirp = opendir(directorypath.c_str());
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

    while ((dp = readdir(dirp)) != NULL) {
        // load the files in a directory
        pcl::PLYReader reader;
        //if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (dp->d_name, *cloud) == -1) {
        if (reader.read<pcl::PointXYZRGBA>(dp->d_name, *cloud)) {
            PCL_ERROR ("Couldn't read file \n");
            continue;
        }
        // cloud loaded; compress to output
        compressFrame(cloud);
        std::cout << "Compressed frame " << dp->d_name << ".\n";
    }

    // what the hell is the next line???? O_o
    (void)closedir(dirp);
}


void PointCloudCompressor::compressFrame(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
    // stringstream to store compressed point cloud
    std::stringstream compressedData;
    std::string * datastring;
    int * header;

    // compress point cloud
    PointCloudEncoder->encodePointCloud(cloud, compressedData);

    // output pointcloud
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    // decompress point cloud
    //PointCloudDecoder->decodePointCloud (compressedData, cloudOut);

    // convert stringstream to string
    datastring = new string(compressedData.str());

    // make frame header (length of frame data in bytes as a binary string)
    header = new int(sizeof(datastring));

    // add newly compressed data to output file with header first
    fwrite(header, sizeof(int), sizeof(header), outfilestream);
    fwrite(datastring, sizeof(char), sizeof(datastring), outfilestream);

    delete datastring;
    delete header;
}


int main(int argc, char **argv) {
    int compression;
    std::string directory, filepath;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("compression,c", po::value<int>(&compression)->default_value(3), "set compression level")
        ("directory,d", po::value<std::string>(&directory), "path to point cloud file directory")
        ("filepath,f", po::value<std::string>(&filepath), "path to compressed point cloud binary file")
    ;

    po::positional_options_description p;

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    po::notify(vm);

    if (vm.count("help") || !vm.count("directory") || !vm.count("filepath")) {
        cout << desc << "\n";
        return 1;
    }

    PointCloudCompressor pccomp(filepath, static_cast<pcl::io::compression_Profiles_e>(compression));
    pccomp.compressDirectory(directory);


    return 0;
}
