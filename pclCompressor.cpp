#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include <stdio.h>
#include <sstream>
#include <stdlib.h>
#include <cstdio>
#include <string>
#include <bitset>
#include <dirent.h>
#include <fstream>

#include <boost/program_options.hpp>

namespace po = boost::program_options;

/*
// DECOMPRESSOR
class PointCloudDeompressor {
    public:
        // constructor
        PointCloudDecompressor(std::string drz_file);

        //destructor
        ~PointCloudDecompressor() {
            infilestream.close();
            delete (PointCloudDecoder);
        };

        // member functions
        std::string getFilepath(void);
        void compressDirectory(const std::string directorypath);
        void compressFrame(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);

    private:
        const pcl::io::compression_Profiles_e compressionProfile;
        pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> * PointCloudEncoder;
        const std::string filepath;
        std::ofstream outfilestream;
};


// CONSTRUCTOR
PointCloudCompressor::PointCloudCompressor(std::string str, pcl::io::compression_Profiles_e compprof, bool stats):
    filepath(str),
    {
    //outfilestream = fopen(filepath.c_str(), "a+b");
    outfilestream.open(filepath.c_str(), std::ofstream::binary);
    PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compressionProfile, showStatistics);
}


// ACCESSOR METHODS
std::string PointCloudCompressor::getFilepath(void) {
    return filepath;
}


// METHODS
void PointCloudCompressor::compressDirectory(const std::string directorypath) {
    struct dirent *dp;

    if (chdir(directorypath.c_str()) == -1) {
         std::cout << "Cannot chdir into " << directorypath << std::endl;
         return;
    }

    DIR * dirp = opendir(directorypath.c_str());
    std::cout << "Compressing PCD files in " << directorypath << std::endl;
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

    while ((dp = readdir(dirp)) != NULL) {
        // load the files in a directory
        std::cout << dp->d_name << std::endl;
        if (strcmp(dp->d_name, ".") == 0)
            continue;
        else if (strcmp(dp->d_name, "..") == 0)
            continue;
        else if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (dp->d_name, *cloud) == -1) {
            PCL_ERROR ("Couldn't read file \n");
            continue;
        }
        // cloud loaded; compress to output
        compressFrame(cloud);
        std::cout << "Compressed frame " << dp->d_name << ".\n";
    }

    (void)closedir(dirp);
}


void PointCloudCompressor::compressFrame(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
    PointCloudEncoder->encodePointCloud(cloud, outfilestream);
}
*/


// COMPRESSOR
class PointCloudCompressor {
    public:
        // constructor
        PointCloudCompressor(std::string str, pcl::io::compression_Profiles_e compprof, bool stats);  // all of the above

        //destructor
        ~PointCloudCompressor() {
            outfilestream.close();
            delete (PointCloudEncoder);
        };

        // member functions
        std::string getFilepath(void);
        void compressDirectory(const std::string directorypath);
        void compressFrame(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);

    private:
        pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> * PointCloudEncoder;
        const std::string filepath;
        std::ofstream outfilestream;
};


// CONSTRUCTOR
PointCloudCompressor::PointCloudCompressor(std::string str, pcl::io::compression_Profiles_e compprof, bool stats):
    filepath(str)
    {
    //outfilestream = fopen(filepath.c_str(), "a+b");
    outfilestream.open(filepath.c_str(), std::ofstream::binary);
    PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compprof, stats);
}


// ACCESSOR METHODS
std::string PointCloudCompressor::getFilepath(void) {
    return filepath;
}


// METHODS
void PointCloudCompressor::compressDirectory(const std::string directorypath) {
    struct dirent *dp;

    if (chdir(directorypath.c_str()) == -1) {
         std::cout << "Cannot chdir into " << directorypath << std::endl;
         return;
    }

    DIR * dirp = opendir(directorypath.c_str());
    std::cout << "Compressing PCD files in " << directorypath << std::endl;
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

    while ((dp = readdir(dirp)) != NULL) {
        // load the files in a directory
        std::cout << dp->d_name << std::endl;
        if (strcmp(dp->d_name, ".") == 0)
            continue;
        else if (strcmp(dp->d_name, "..") == 0)
            continue;
        else if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (dp->d_name, *cloud) == -1) {
            PCL_ERROR ("Couldn't read file \n");
            continue;
        }
        // cloud loaded; compress to output
        compressFrame(cloud);
        std::cout << "Compressed frame " << dp->d_name << ".\n";
    }

    (void)closedir(dirp);
}


void PointCloudCompressor::compressFrame(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
    std::stringstream tempstream;
    long unsigned int streamsize;

    PointCloudEncoder->encodePointCloud(cloud, tempstream);

    streamsize = static_cast<long unsigned int>(tempstream.tellp());
    std::cout << streamsize << ", " << tempstream.tellp() << std::endl;
    outfilestream.write(reinterpret_cast<const char *>(&streamsize), sizeof(streamsize));
    outfilestream << tempstream.rdbuf();
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

    PointCloudCompressor pccomp(filepath, static_cast<pcl::io::compression_Profiles_e>(compression), true);
    pccomp.compressDirectory(directory);


    return 0;
}
