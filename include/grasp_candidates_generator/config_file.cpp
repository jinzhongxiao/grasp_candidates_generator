#include <iostream>
#include <string>
#include <sstream>
#include <map>
#include <fstream>
#include <typeinfo>


class Convert
{
  public:
    // Convert T, which should be a primitive, to a std::string.
    template <typename T>
    static std::string T_to_string(T const &val)
    {
      std::ostringstream ostr;
      ostr << val;

      return ostr.str();
    }

    // Convert a std::string to T.
    template <typename T>
    static T string_to_T(std::string const &val)
    {
      std::istringstream istr(val);
      T returnVal;
      if (!(istr >> returnVal))
        std::cout << "CFG: Not a valid " + (std::string)typeid(T).name() + " received!\n";

      return returnVal;
    }

//    template <typename T>
//    static std::string string_to_T(std::string const &val)
//    {
//      return val;
//    }
};


class ConfigFile
{
  private:

    // Remove comment from a single line.
    void removeComment(std::string &line) const
    {
      if (line.find('#') != line.npos)
        line.erase(line.find('#'));
    }

    bool onlyWhitespace(const std::string &line) const
    {
      return (line.find_first_not_of(' ') == line.npos);
    }

    bool validLine(const std::string &line) const
    {
      std::string temp = line;
      temp.erase(0, temp.find_first_not_of("\t "));
      if (temp[0] == '=')
        return false;

      for (size_t i = temp.find('=') + 1; i < temp.length(); i++)
        if (temp[i] != ' ')
          return true;

      return false;
    }

    void extractKey(std::string &key, size_t const &sepPos, const std::string &line) const
    {
      key = line.substr(0, sepPos);
      if (key.find('\t') != line.npos || key.find(' ') != line.npos)
        key.erase(key.find_first_of("\t "));
    }

    void extractValue(std::string &value, size_t const &sepPos, const std::string &line) const
    {
      value = line.substr(sepPos + 1);
      value.erase(0, value.find_first_not_of("\t "));
      value.erase(value.find_last_not_of("\t ") + 1);
    }

    void extractContents(const std::string &line)
    {
      std::string temp = line;
      // Erase leading whitespace from the line.
      temp.erase(0, temp.find_first_not_of("\t "));
      size_t sepPos = temp.find('=');

      std::string key, value;
      extractKey(key, sepPos, temp);
      extractValue(value, sepPos, temp);

      if (!keyExists(key))
        contents.insert(std::pair<std::string, std::string>(key, value));
      else
        std::cout << "CFG: Can only have unique key names!\n";
    }

    // lineNo = the current line number in the file.
    // line = the current line, with comments removed.
    void parseLine(const std::string &line, size_t const lineNo)
    {
      if (line.find('=') == line.npos)
        std::cout << "CFG: Couldn't find separator on line: " + Convert::T_to_string(lineNo) + "\n";

      if (!validLine(line))
        std::cout << "CFG: Bad format for line: " + Convert::T_to_string(lineNo) + "\n";

      extractContents(line);
    }

    void ExtractKeys()
    {
      std::ifstream file;
      file.open(fName.c_str());
      if (!file)
        std::cout << "CFG: File " + fName + " couldn't be found!\n";

      std::string line;
      size_t lineNo = 0;
      while (std::getline(file, line))
      {
        lineNo++;
        std::string temp = line;

        if (temp.empty())
          continue;

        removeComment(temp);
        if (onlyWhitespace(temp))
          continue;

        parseLine(temp, lineNo);
      }

      file.close();
    }

    std::map<std::string, std::string> contents;
    std::string fName;

  public:

    ConfigFile(const std::string &fName)
    {
      this->fName = fName;
      ExtractKeys();
    }

    // Check if a given key exists in the configuration file.
    bool keyExists(const std::string &key) const
    {
      return contents.find(key) != contents.end();
    }

    // Retrieve the value of a given key.
    template <typename ValueType>
    ValueType getValueOfKey(const std::string &key, ValueType const &defaultValue = ValueType()) const
    {
      if (!keyExists(key))
        return defaultValue;

      return Convert::string_to_T<ValueType>(contents.find(key)->second);
    }

    std::string getValueOfKeyAsString(const std::string &key, const std::string &defaultValue)
    {
      if (!keyExists(key))
        return defaultValue;

      return contents.find(key)->second;
    }
};
