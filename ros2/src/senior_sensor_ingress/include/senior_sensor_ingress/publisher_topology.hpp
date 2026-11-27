#pragma once

#include <string>

namespace senior_sensor_ingress {
// Callers supply their one existing discovery snapshot; no MessageInfo GID gate.
template<typename Endpoints>
bool publisher_matches(const Endpoints & endpoints, const std::string & name,
                       const std::string & ns, const std::string & type = {})
{
  return endpoints.size() == 1 &&
    (name.empty() || endpoints.front().node_name() == name) &&
    (ns.empty() || endpoints.front().node_namespace() == ns) &&
    (type.empty() || endpoints.front().topic_type() == type);
}
}  // namespace senior_sensor_ingress
