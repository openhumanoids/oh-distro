#include <boost/filesystem/operations.hpp>

namespace action_authoring {

class UtilityFile 
{
public:
    static bool hasEnding (std::string const &fullString, std::string const &ending)
    {
        if (fullString.length() >= ending.length()) {
            return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
        } else {
            return false;
        }
    }

    static std::vector<std::string> getFilenamesInDirectory(std::string directoryPath, std::string requiredExtension)
    {
        std::vector<std::string> filenames;
        namespace fs = boost::filesystem;
        fs::path someDir(directoryPath);
        fs::directory_iterator end_iter;

        if (fs::exists(someDir) && fs::is_directory(someDir))
        {
            for (fs::directory_iterator dir_iter(someDir) ; dir_iter != end_iter ; ++dir_iter)
            {
                if (fs::is_regular_file(dir_iter->status()) && 
                    UtilityFile::hasEnding(dir_iter->path().native(), requiredExtension))
                {
                    filenames.push_back(dir_iter->path().native());
                }
            }
        }
        return filenames;
    }

}; // class UtilityFile
    
} // namespace action_authoring
