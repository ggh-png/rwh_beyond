// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#ifndef XML_TREE_FILTER_HPP_
#define XML_TREE_FILTER_HPP_

#include <filesystem>
#include <map>
#include <set>
#include <string>
#include <utility>

namespace rwh_beyond_navigator
{
// Private runtime copies: never rewrite the editor's XML. Stable paths for
// unchanged content retain Nav2's tree cache; changed files get a new revision.
class XmlTreeFilter
{
public:
  XmlTreeFilter() = default;
  ~XmlTreeFilter();
  XmlTreeFilter(const XmlTreeFilter &) = delete;
  XmlTreeFilter & operator=(const XmlTreeFilter &) = delete;
  std::string prepare(const std::string & source);
  size_t removed_attributes() const {return removed_;}

private:
  std::filesystem::path prepare_file(
    const std::filesystem::path & source, std::set<std::filesystem::path> & visiting);
  std::filesystem::path directory_;
  std::map<std::filesystem::path, std::pair<std::string, std::filesystem::path>> cache_;
  size_t revision_{0}, removed_{0};
};
}  // namespace rwh_beyond_navigator
#endif  // XML_TREE_FILTER_HPP_
