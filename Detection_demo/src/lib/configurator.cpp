#include "configurator.h"

using namespace std;


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~ Class functions ~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ConfigReader* ConfigReader::m_pInstance = NULL;

ConfigReader::ConfigReader()
{
    m_ConfigSettingMap.clear();
}

ConfigReader::~ConfigReader()
{
    m_ConfigSettingMap.clear();
}

ConfigReader* ConfigReader::getInstance()
{
    // No need to use double re-check lock mechanism here
    // because this getInstance() will call at the time of
    // initialization only and mostly, at the time of
    // initialization, there will be only one thread.
    if(m_pInstance == NULL)
        m_pInstance = new ConfigReader;
    
    return m_pInstance;
}

bool ConfigReader::getValue(std::string tag, int& value)
{
    map<string, string>::iterator it;
    it = m_ConfigSettingMap.find(tag);
    if(it != m_ConfigSettingMap.end())
    {
        value = atoi((it->second).c_str());
        return true;
    }
    return false;
}

bool ConfigReader::getValue(std::string tag, float& value)
{
    map<string, string>::iterator it;
    it = m_ConfigSettingMap.find(tag);
    if(it != m_ConfigSettingMap.end())
    {
        value = atof((it->second).c_str());
        return true;
    }
    return false;
}

bool ConfigReader::getValue(std::string tag, std::string& value)
{
    map<string, string>::iterator it;
    it = m_ConfigSettingMap.find(tag);
    if(it != m_ConfigSettingMap.end())
    {
        value = it->second;
        return true;
    }
    return false;
}

bool ConfigReader::getValue(std::string tag, bool& value)
{
    map<string, string>::iterator it;
    it = m_ConfigSettingMap.find(tag);
    if(it != m_ConfigSettingMap.end())
    {
        value = atof((it->second).c_str());
        return true;
    }
    return false;
}

// bool ConfigReader::getValue(std::string tag, Eigen::Vector3f& value)
// {
//     map<string, string>::iterator it;
    
//     it = m_ConfigSettingMap.find(tag);
//     ushort strBegin = 0, strEnd = 0;
//     if(it != m_ConfigSettingMap.end())
//     {
//         for (ushort i=0; i<3; i++)
//         {
//             strEnd = it->second.find_first_of(",", strBegin);
//             value[i] = atof(trim(it->second.substr(strBegin, strEnd - strBegin)).c_str());
//             strBegin = strEnd+1;
//         }
//         return true;
//     }
//     return false;
// }

bool ConfigReader::getValue(std::string tag, cv::Scalar& value)
{
    map<string, string>::iterator it;
    it = m_ConfigSettingMap.find(tag);
    ushort strBegin = 0, strEnd = 0;
    if(it != m_ConfigSettingMap.end())
    {
        for (ushort i=0; i<3; i++)
        {
            strEnd = it->second.find_first_of(",", strBegin);
            value[i] = atof(trim(it->second.substr(strBegin, strEnd - strBegin)).c_str());
            strBegin = strEnd+1;
        }
        return true;
    }
    return false;
}

bool ConfigReader::getValue(std::string tag, cv::Size& value)
{
    map<string, string>::iterator it;
    it = m_ConfigSettingMap.find(tag);
    ushort strBegin = 0, strEnd = 0;
    if(it != m_ConfigSettingMap.end())
    {
        for (ushort i=0; i<2; i++)
        {
            strEnd = it->second.find_first_of(",", strBegin);
            value.width = atoi(trim(it->second.substr(strBegin, strEnd - strBegin)).c_str());
            strBegin = strEnd+1;
        }
        value.height = value.width;
        return true;
    }
    return false;
}

bool ConfigReader::parseFile(string fileName)
{
    ifstream inputFile;
    inputFile.open(fileName.c_str());
    string delimiter = "=";
    int initPos = 0;
    if (inputFile.fail())
    {
        cout << "Unable to find defaultConfig file" << endl;
        return false;
    }

    string line;
    while(getline(inputFile, line))
    {
        // Remove comment Lines
        size_t found = line.find_first_of('#');
        string configData = line.substr(0, found);

        // Remove ^M from configData
        configData.erase(std::remove(configData.begin(), configData.end(), '\r'), configData.end());

        if (configData.empty())
            continue;
        
        unsigned int length = configData.find(delimiter);

        string tag, value;

        if (length != string::npos)
        {
            tag   = configData.substr(initPos, length);
            value = configData.substr(length+1);
        }

        // Trim white spaces
        tag   = reduce(tag);
        value = reduce(value);

        if(tag.empty() || value.empty())
            continue;

        // Check if any of the tags is repeated more than one times
        // it needs to pick the latest one instead of the old one.

        // Search, if the tag is already present or not
        // If it is already present, then delete an existing one

        std::map<std::string, std::string>::iterator itr = m_ConfigSettingMap.find(tag);
        if(itr != m_ConfigSettingMap.end())
        {
            m_ConfigSettingMap.erase(tag);
        }

        m_ConfigSettingMap.insert(std::pair<string, string>(tag, value));
    }
    return true;
}

std::string ConfigReader::trim(const std::string& str, const std::string& whitespace)
{
    size_t strBegin = str.find_first_not_of(whitespace);
    if (strBegin == std::string::npos)
        return "";

    size_t strEnd = str.find_last_not_of(whitespace);
    size_t strRange = strEnd - strBegin + 1;

    return str.substr(strBegin, strRange);
}

std::string ConfigReader::reduce(const std::string& str,
            const std::string& fill,
            const std::string& whitespace)
{
    // trim first
    string result = trim(str, whitespace);

    // replace sub ranges
    size_t beginSpace = result.find_first_of(whitespace);
    while (beginSpace != std::string::npos)
    {
        size_t endSpace = result.find_first_not_of(whitespace, beginSpace);
        size_t range = endSpace - beginSpace;

        result.replace(beginSpace, range, fill);

        size_t newStart = beginSpace + fill.length();
        beginSpace = result.find_first_of(whitespace, newStart);
    }

    return result;
}

void ConfigReader::dumpFileValues()
{
   map<string, string>::iterator it;
   for(it=m_ConfigSettingMap.begin(); it!=m_ConfigSettingMap.end(); ++it)
   {
      cout << it->first << " = " << it->second << endl;
   }
}