#ifndef _JSON_H
#define _JSON_H

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <cctype>
#include <stdexcept>

class JSON {
public:
    enum class Type { STRING, NUMBER, BOOLEAN, OBJECT, ARRAY, NULLTYPE };

    struct Value {
        Type type;
        std::string stringValue;
        double numberValue;
        bool booleanValue;
        std::map<std::string, Value> objectValue;
        std::vector<Value> arrayValue;

        Value() : type(Type::NULLTYPE) {}
    };

    static Value parse(const std::string& jsonStr);

private:
    static size_t skipWhitespace(const std::string& str, size_t pos);
    static Value parseValue(const std::string& str, size_t& pos);
    static Value parseObject(const std::string& str, size_t& pos);
    static Value parseArray(const std::string& str, size_t& pos);
    static Value parseString(const std::string& str, size_t& pos);
    static Value parseNumber(const std::string& str, size_t& pos);
    static Value parseBoolean(const std::string& str, size_t& pos);
    static Value parseNull(const std::string& str, size_t& pos);
};


#endif

