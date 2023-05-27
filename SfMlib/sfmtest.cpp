#include "Common.h"
#include "SfM.h"
#include <boost/filesystem.hpp>
#include <string>
#include <vector>

int main(int argc, char **argv)
{

    using namespace boost::filesystem;
    std::string directoryPath = "";
    std::string outputPath = "output";
    if (argc == 1)
    {
        cout << "please use 'SfMTest [imagePath] [outputPath](optional)' ";
        return 0;
    }
    else if (argc == 2)
    {
        directoryPath = argv[1];
    }
    else
    {
        directoryPath = argv[1];
        outputPath = argv[2];
    }

    vector<string> Filenames;
    path dirPath(directoryPath);
    if (not exists(dirPath) or not is_directory(dirPath))
    {
        cerr << "Cannot open directory: " << directoryPath << endl;
        return false;
    }

    for (directory_entry &x : directory_iterator(dirPath))
    {
        string extension = x.path().extension().string();
        boost::algorithm::to_lower(extension);
        if (extension == ".jpg" or extension == ".png")
        {
            Filenames.push_back(x.path().string());
        }
    }

    if (Filenames.size() <= 0)
    {
        cerr << "Unable to find valid files in images directory (\"" << directoryPath << "\")." << endl;
        return false;
    }

    SfM a(Filenames);
    a.sfmStart();
    a.saveResultForMVS(outputPath);
    a.saveCloudAndCamerasToPLY(outputPath, "hello");
    return 0;
}