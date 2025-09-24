#ifndef CAPRA_JOY_CONTROLS__UTILS_HPP_
#define CAPRA_JOY_CONTROLS__UTILS_HPP_

#include <vector>
#include <stdexcept>
#include <sstream>



namespace capra_joy_controls {

template <typename TIter, typename TValue>
inline int index_of(TIter begin, TIter end, const TValue& val) {
    if (auto match = std::find(begin, end, val); match != end) {
        return (int)(match - begin);
    }
    return -1;
}

template <typename T>
inline std::string join(const std::vector<T>& parts, const std::string& sep) {
    std::ostringstream oss;
    for (size_t i = 0; i < parts.size(); ++i) {
        if (i > 0) oss << sep;
        oss << parts[i];
    }
    return oss.str();
}

inline std::vector<std::string> split(const std::string& sep, const std::string& text) {
    if (sep.empty()) return {text};

    std::vector<std::string> res;
    auto selItem = text.cbegin();

    for (auto selBegin = text.cbegin(); selBegin + sep.size() <= text.cend(); ++selBegin) {
        if (std::equal(sep.cbegin(), sep.cend(), selBegin)) {
            res.emplace_back(selItem, selBegin);
            selBegin += sep.size() - 1;  // jump to end of separator
            selItem = selBegin + 1;      // next token starts after separator
        }
    }
    // add trailing text
    res.emplace_back(selItem, text.cend());

    return res;
}



} // capra_joy_controls

#endif // CAPRA_JOY_CONTROLS__UTILS_HPP_