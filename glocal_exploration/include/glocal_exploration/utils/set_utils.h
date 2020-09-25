#ifndef GLOCAL_EXPLORATION_UTILS_SET_UTILS_H_
#define GLOCAL_EXPLORATION_UTILS_SET_UTILS_H_

#include <algorithm>
#include <set>

namespace glocal_exploration::set_utils {

template <typename KeyType>
inline std::set<KeyType> setDifference(const std::set<KeyType>& positive_set,
                                       const std::set<KeyType>& negative_set) {
  std::set<KeyType> result_set;
  std::set_difference(positive_set.begin(), positive_set.end(),
                      negative_set.begin(), negative_set.end(),
                      std::inserter(result_set, result_set.end()));
  return result_set;
}

template <typename KeyType>
inline std::set<KeyType> setIntersection(const std::set<KeyType>& first_set,
                                         const std::set<KeyType>& second_set) {
  std::set<KeyType> result_set;
  std::set_intersection(first_set.begin(), first_set.end(), second_set.begin(),
                        second_set.end(),
                        std::inserter(result_set, result_set.end()));
  return result_set;
}

}  // namespace glocal_exploration::set_utils

#endif  // GLOCAL_EXPLORATION_UTILS_SET_UTILS_H_
