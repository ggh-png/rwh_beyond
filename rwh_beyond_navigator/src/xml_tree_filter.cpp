// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include "xml_tree_filter.hpp"

#include <tinyxml2.h>

#include <cstdlib>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <system_error>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace rwh_beyond_navigator
{
XmlTreeFilter::~XmlTreeFilter()
{
  if (!directory_.empty()) {
    std::error_code error;
    std::filesystem::remove_all(directory_, error);
  }
}
std::string XmlTreeFilter::prepare(const std::string & source)
{
  if (directory_.empty()) {
    auto pattern = (std::filesystem::temp_directory_path() / "rwh-bt-xml-XXXXXX").string();
    std::vector<char> buffer(pattern.begin(), pattern.end());
    buffer.push_back('\0');
    auto directory = ::mkdtemp(buffer.data());
    if (!directory) {throw std::runtime_error("Cannot create BT XML runtime directory");}
    directory_ = directory;
  }
  removed_ = 0;
  std::set<std::filesystem::path> visiting;
  return prepare_file(source, visiting).string();
}
std::filesystem::path XmlTreeFilter::prepare_file(
  const std::filesystem::path & source, std::set<std::filesystem::path> & visiting)
{
  const auto path = std::filesystem::canonical(source);
  if (!visiting.insert(path).second) {
    throw std::runtime_error("Cyclic BT XML include: " + path.string());
  }
  tinyxml2::XMLDocument document;
  if (document.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Cannot parse BT XML " + path.string() + ": " + document.ErrorStr());
  }
  if (!document.RootElement()) {throw std::runtime_error("BT XML has no root: " + path.string());}
  std::function<void(tinyxml2::XMLElement *)> filter = [&](tinyxml2::XMLElement * element) {
      for (auto attr = element->FirstAttribute(); attr; ) {
        auto next = attr->Next();
        if (std::string(attr->Value()).empty()) {
          element->DeleteAttribute(attr->Name());
          ++removed_;
        }
        attr = next;
      }
      if (std::string(element->Name()) == "include") {
        const char * included = element->Attribute("path");
        if (!included) {throw std::runtime_error("BT XML include has no path: " + path.string());}
        std::filesystem::path include_path(included);
        if (include_path.is_relative()) {
          auto package = element->Attribute("ros_pkg");
          include_path = (package ?
            std::filesystem::path(ament_index_cpp::get_package_share_directory(package)) :
            path.parent_path()) / include_path;
        }
        const auto clean = prepare_file(include_path, visiting);
        element->SetAttribute("path", clean.c_str());
        // The runtime include is absolute; package resolution has already happened.
        element->DeleteAttribute("ros_pkg");
      }
      for (auto child = element->FirstChildElement(); child; child = child->NextSiblingElement()) {
        filter(child);
      }
    };
  filter(document.RootElement());
  visiting.erase(path);
  tinyxml2::XMLPrinter printer;
  document.Print(&printer);
  const std::string text(printer.CStr());
  auto cached = cache_.find(path);
  if (cached != cache_.end() && cached->second.first == text) {return cached->second.second;}
  const auto output = directory_ / ("tree-" + std::to_string(++revision_) + ".xml");
  std::ofstream stream;
  stream.exceptions(std::ios::badbit | std::ios::failbit);
  stream.open(output);
  stream << text;
  stream.close();
  cache_[path] = {text, output};
  return output;
}
}  // namespace rwh_beyond_navigator
