#include "./lib/followme.h"
#include "./lib/configurator.h"


using namespace std;



// --------------------------------------------
// ------------------ main --------------------
// --------------------------------------------
int main (int argc, char** argv)
{
    // Create object of the class ConfigReader
    ConfigReader* p = ConfigReader::getInstance();

    // parse the configuration file
    p->parseFile();

    // Dump map on the console after parsing it (for debug)
    // p->dumpFileValues();

    // Print divider on the console to understand the output properly
    cout << endl << "=================================================" << endl << endl;

    // Define variables to store the value
    string name("");
    string skill("");
    string website("");
    float age = 0;

    // Update the variable by the value present in the configuration file.
    p->getValue("Name", name);
    p->getValue("Skill", skill);
    p->getValue("Website", website);
    p->getValue("Age", age);

    // Variables has been updated. Now print it on the console.
    cout << "Name    = " << name << endl;
    cout << "Skill   = " << skill << endl;
    cout << "Website = " << website << endl;
    cout << "Age     = " << age << endl;

    return 0;
}