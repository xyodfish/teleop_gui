#pragma once
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace ConfigParser {

    struct ConfigValue {
        std::string key;
        std::string value;
        std::string type;
        std::vector<ConfigValue> children;
    };

    class YamlConfig {
       public:
        bool loadFromFile(const std::string& filename) {
            try {
                config_ = YAML::LoadFile(filename);
                return true;
            } catch (const YAML::Exception& e) {
                std::cerr << "Failed to load config file: " << e.what() << std::endl;
                return false;
            }
        }

        bool loadFromString(const std::string& content) {
            try {
                config_ = YAML::Load(content);
                return true;
            } catch (const YAML::Exception& e) {
                std::cerr << "Failed to parse YAML: " << e.what() << std::endl;
                return false;
            }
        }

        std::vector<ConfigValue> parseToTree() {
            std::vector<ConfigValue> result;
            if (config_.IsMap()) {
                parseMap(config_, "", result);
            }
            return result;
        }

        std::string getValue(const std::string& path, const std::string& defaultValue = "") {
            try {
                YAML::Node node = config_;
                std::istringstream iss(path);
                std::string token;

                while (std::getline(iss, token, '.')) {
                    if (node.IsMap() && node[token]) {
                        node = node[token];
                    } else {
                        return defaultValue;
                    }
                }

                return node.as<std::string>();
            } catch (const YAML::Exception& e) { return defaultValue; }
        }

       private:
        void parseMap(const YAML::Node& node, const std::string& parentKey, std::vector<ConfigValue>& result) {
            for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
                ConfigValue cv;
                std::string key = it->first.as<std::string>();
                cv.key          = parentKey.empty() ? key : parentKey + "." + key;

                if (it->second.IsMap()) {
                    cv.type = "map";
                    parseMap(it->second, cv.key, cv.children);
                } else if (it->second.IsSequence()) {
                    cv.type = "sequence";
                    parseSequence(it->second, cv.key, cv.children);
                } else {
                    cv.type = "value";
                    try {
                        cv.value = it->second.as<std::string>();
                    } catch (const YAML::Exception& e) { cv.value = "<binary data>"; }
                }
                result.push_back(cv);
            }
        }

        void parseSequence(const YAML::Node& node, const std::string& parentKey, std::vector<ConfigValue>& result) {
            int index = 0;
            for (const auto& item : node) {
                ConfigValue cv;
                cv.key = parentKey + "[" + std::to_string(index++) + "]";

                if (item.IsMap()) {
                    cv.type = "map";
                    parseMap(item, cv.key, cv.children);
                } else if (item.IsSequence()) {
                    cv.type = "sequence";
                    parseSequence(item, cv.key, cv.children);
                } else {
                    cv.type = "value";
                    try {
                        cv.value = item.as<std::string>();
                    } catch (const YAML::Exception& e) { cv.value = "<binary data>"; }
                }
                result.push_back(cv);
            }
        }

        YAML::Node config_;
    };

}  // namespace ConfigParser