#include "capra_joy_controls/parsable/SchemeMap.hpp"

namespace capra_joy_controls::parsable
{

void SchemeMap::parse_from(const YAML::Node &node)
{
    expect_not_null(node);
    expect_defined(node);

    for (auto ncontrolScheme : node) {
        auto c = ncontrolScheme.second;
        c.SetTag(ncontrolScheme.first.Scalar());
        controlSchemes.emplace_back(c);
    }
}

} // namespace capra_joy_controls::parsable

