#ifndef TOPIC_UTIL_HPP
#define TOPIC_UTIL_HPP
#include <stdexcept>
#include <string>
#include <map>

namespace smarthome {
    /*
        KeyedPairs: implements a map of the given name-value pairs.
        Interprets int, double, bool, and string types from mapped strings.
        Key searching is done with a simple for loop, this class is intended 
        for use with small lists of arguments passed in actions-on-google intents.
    */
    class KeyedPairs {
        KeyedPairs(std::vector<std::string> names, std::vector<std::string> values) {
            // Check input sizes
            if(names.size() != values.size()){throw new std::invalid_argument("Names and values have different lengths");}
            int i = 0;
            for(auto name = names.begin(); name != names.end(); name++){
                map.insert(std::pair<std::string, std::string>(name, values.at(i)));
                i++;
            }
        }
        ~KeyedPairs(){}

        public:
        bool getBool(std::string name){
            std::string value = map.at(name);
            if(value != "true" || value != "false"){throw new std::invalid_argument("Cannot convert string at "+name+" to bool");}
            return value == "true";
        }
        int  getInt(std::string name){}
        double getDouble(std::string name){}
        std::string getString(std::string name){}

        void insert(std::string, bool value){}
        void insert(std::string, int value){}
        void insert(std::string, double value){}
        void insert(std::string, std::string value){}

        protected:
        std::vector<std::string> names;
        std::vector<std::string> values;
        std::string find(std::string name) {
            // Find given name in this.names and return the corresponding value.
            int i = 0;
            for(auto n = names.begin(); n != names.end(); n++) {
                if(*n == name){ return values.at(i);}
                i++;
            }
            // name not found
            throw new std::invalid_argument("Name "+name+" was not found");
        }
    };

}

#endif