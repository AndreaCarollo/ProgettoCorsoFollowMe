// Double inclusion guard
#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H

#include "followme.h"
#include <map>
#include <fstream>
#include <algorithm>

class ConfigReader
{
    private:
        // Define the map to store data from the config file
        std::map<std::string, std::string> m_ConfigSettingMap;

        // Static pointer instance to make this class singleton.
        static ConfigReader* m_pInstance;

    public:
        // Public static method getInstance(). This function is
        // responsible for object creation.
        static ConfigReader* getInstance();

        // Parse the config file.
        bool parseFile(std::string fileName = "../config.ini");

        // Overloaded getValue() function.
        // Value of the tag in cofiguration file could be
        // string or integer. So the caller need to take care this.
        // Caller need to call appropiate function based on the
        // data type of the value of the tag.

        bool getValue(std::string tag, int& value);
        bool getValue(std::string tag, float& value);
        bool getValue(std::string tag, std::string& value);
        bool getValue(std::string tag, Eigen::Vector3f& value);
        bool getValue(std::string tag, cv::Scalar& value);

        // Function dumpFileValues is for only debug purpose
        void dumpFileValues();

    private:
        // Define constructor in the private section to make this
        // class as singleton.
        ConfigReader();

        // Define destructor in private section, so no one can delete 
        // the instance of this class.
        ~ConfigReader();

        // Define copy constructor in the private section, so that no one can 
        // violate the singleton policy of this class
        ConfigReader(const ConfigReader& obj){}
         
        // Define assignment operator in the private section, so that no one can 
        // violate the singleton policy of this class
        void operator=(const ConfigReader& obj){}

        // Helper function to trim the tag and value. These helper function is
        // calling to trim the un-necessary spaces.
        std::string trim(const std::string& str, const std::string& whitespace = " \t");
        std::string reduce(const std::string& str,
                const std::string& fill = " ",
                const std::string& whitespace = " \t");

};


#endif