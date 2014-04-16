#ifndef STARMAC_ROBOTS_ASCTEC_MISC_H
#define STARMAC_ROBOTS_ASCTEC_MISC_H

#include <algorithm>

namespace starmac_robots_asctec
{
template<typename Tx, typename Tt>
  void limitSlewRate(Tx x, Tx x_prev, Tt dt, Tx rate_limit, Tx& x_limited)
  {
    Tx limit = rate_limit * dt;
    x_limited = max(x_prev - limit, min(x_prev + limit, x));
  }
}
#endif
