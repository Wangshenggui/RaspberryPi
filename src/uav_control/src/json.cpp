#include "json.h"

// 跳过空白字符
size_t JSON::skipWhitespace(const std::string& str, size_t pos) {
    while (pos < str.size() && std::isspace(str[pos])) {
        pos++;
    }
    return pos;
}

// 解析不同类型的值
JSON::Value JSON::parseValue(const std::string& str, size_t& pos) {
    pos = skipWhitespace(str, pos);
    if (pos >= str.size()) {
        throw std::invalid_argument("Unexpected end of input");
    }

    char c = str[pos];
    if (c == '"') return parseString(str, pos);
    if (c == '{') return parseObject(str, pos);
    if (c == '[') return parseArray(str, pos);
    if (std::isdigit(c) || c == '-') return parseNumber(str, pos);
    if (c == 't' || c == 'f') return parseBoolean(str, pos);
    if (c == 'n') return parseNull(str, pos);

    throw std::invalid_argument("Unexpected character");
}

// 解析字符串
JSON::Value JSON::parseString(const std::string& str, size_t& pos) {
    Value value;
    value.type = Type::STRING;
    pos++;  // Skip opening quote

    while (pos < str.size() && str[pos] != '"') {
        value.stringValue += str[pos++];
    }
    pos++;  // Skip closing quote
    return value;
}

// 解析数字
JSON::Value JSON::parseNumber(const std::string& str, size_t& pos) {
    Value value;
    value.type = Type::NUMBER;

    size_t start = pos;
    while (pos < str.size() && (std::isdigit(str[pos]) || str[pos] == '.' || str[pos] == '-')) {
        pos++;
    }
    value.numberValue = std::stod(str.substr(start, pos - start));
    return value;
}

// 解析布尔值
JSON::Value JSON::parseBoolean(const std::string& str, size_t& pos) {
    Value value;
    value.type = Type::BOOLEAN;
    if (str.substr(pos, 4) == "true") {
        value.booleanValue = true;
        pos += 4;
    } else if (str.substr(pos, 5) == "false") {
        value.booleanValue = false;
        pos += 5;
    } else {
        throw std::invalid_argument("Invalid boolean value");
    }
    return value;
}

// 解析 null
JSON::Value JSON::parseNull(const std::string& str, size_t& pos) {
    Value value;
    value.type = Type::NULLTYPE;
    if (str.substr(pos, 4) == "null") {
        pos += 4;
    } else {
        throw std::invalid_argument("Invalid null value");
    }
    return value;
}

// 解析对象
JSON::Value JSON::parseObject(const std::string& str, size_t& pos) {
    Value value;
    value.type = Type::OBJECT;

    pos++;  // Skip opening brace
    while (true) {
        pos = skipWhitespace(str, pos);
        if (str[pos] == '}') {
            pos++;
            break;
        }

        // Read key (should be a string)
        Value key = parseString(str, pos);
        pos = skipWhitespace(str, pos);
        if (str[pos] != ':') throw std::invalid_argument("Expected ':' after key");
        pos++;  // Skip colon

        // Read value
        Value val = parseValue(str, pos);
        value.objectValue[key.stringValue] = val;

        pos = skipWhitespace(str, pos);
        if (str[pos] == '}') {
            pos++;
            break;
        }
        if (str[pos] != ',') throw std::invalid_argument("Expected ',' or '}' after key-value pair");
        pos++;  // Skip comma
    }

    return value;
}

// 解析数组
JSON::Value JSON::parseArray(const std::string& str, size_t& pos) {
    Value value;
    value.type = Type::ARRAY;

    pos++;  // Skip opening bracket
    while (true) {
        pos = skipWhitespace(str, pos);
        if (str[pos] == ']') {
            pos++;
            break;
        }

        // Read value
        value.arrayValue.push_back(parseValue(str, pos));

        pos = skipWhitespace(str, pos);
        if (str[pos] == ']') {
            pos++;
            break;
        }
        if (str[pos] != ',') throw std::invalid_argument("Expected ',' or ']' after value");
        pos++;  // Skip comma
    }

    return value;
}

// 解析JSON
JSON::Value JSON::parse(const std::string& jsonStr) {
    size_t pos = 0;
    return parseValue(jsonStr, pos);
}

