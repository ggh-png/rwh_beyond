// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include <tinyxml2.h>

#include <chrono>
#include <filesystem>
#include <iterator>
#include <stdexcept>
#include <fstream>
#include <string>

#include "gtest/gtest.h"
#include "xml_tree_filter.hpp"

namespace fs = std::filesystem;
using rwh_beyond_navigator::XmlTreeFilter;

class XmlFilter : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // A filter instance also provides a private, unique directory for fixtures.
    seed = fs::temp_directory_path() / ("filter-fixture-" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()) + ".xml");
    std::ofstream(seed) << "<root/>";
    directory = fs::path(allocator.prepare(seed.string())).parent_path();
  }
  void TearDown() override {fs::remove(seed);}
  fs::path write(const std::string & name, const std::string & text)
  {
    const auto path = directory / name;
    fs::create_directories(path.parent_path());
    std::ofstream(path) << text;
    return path;
  }
  static std::string read(const fs::path & path)
  {
    std::ifstream input(path);
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  }
  XmlTreeFilter allocator, filter;
  fs::path seed, directory;
};
TEST_F(XmlFilter, RemovesOnlyExactlyEmptyAttributesAndPreservesSourceAndText)
{
  const std::string original =
    "<root><!-- keep --><Node empty=\"\" spaces=\"  \" zero=\"0\" flag=\"false\" "
    "port=\"{capture}\" escaped=\"a&amp;b\">&quot;&quot;</Node></root>";
  auto source = write("tree.xml", original);
  auto cleaned = filter.prepare(source.string());
  tinyxml2::XMLDocument document;
  ASSERT_EQ(document.LoadFile(cleaned.c_str()), tinyxml2::XML_SUCCESS);
  auto node = document.RootElement()->FirstChildElement("Node");
  EXPECT_EQ(node->Attribute("empty"), nullptr);
  EXPECT_STREQ(node->Attribute("spaces"), "  ");
  EXPECT_STREQ(node->Attribute("zero"), "0");
  EXPECT_STREQ(node->Attribute("flag"), "false");
  EXPECT_STREQ(node->Attribute("port"), "{capture}");
  EXPECT_STREQ(node->Attribute("escaped"), "a&b");
  EXPECT_STREQ(node->GetText(), "\"\"");
  EXPECT_EQ(filter.removed_attributes(), 1u);
  EXPECT_EQ(read(source), original);
  EXPECT_NE(read(cleaned).find("<!-- keep -->"), std::string::npos);
  EXPECT_FALSE(fs::exists(source.string() + ".bak"));
}
TEST_F(XmlFilter, UnchangedContentReusesPathButSavedEditsGetNewRevision)
{
  auto source = write("tree.xml", "<root value=\"\"/>");
  auto first = filter.prepare(source.string());
  EXPECT_EQ(first, filter.prepare(source.string()));
  write("tree.xml", "<root value=\"new\"/>");
  EXPECT_NE(first, filter.prepare(source.string()));
  EXPECT_TRUE(fs::exists(first));
}
TEST_F(XmlFilter, ResolvesAndCleansNestedRelativeIncludesAndTracksChildChanges)
{
  auto source = write("tree.xml", "<root><include path=\"nested/child.xml\"/></root>");
  auto child = write("nested/child.xml", "<root><include path=\"leaf.xml\"/></root>");
  auto leaf = write("nested/leaf.xml", "<root><Node optional=\"\"/></root>");
  auto first = filter.prepare(source.string());
  tinyxml2::XMLDocument document;
  ASSERT_EQ(document.LoadFile(first.c_str()), tinyxml2::XML_SUCCESS);
  auto include = document.RootElement()->FirstChildElement("include")->Attribute("path");
  ASSERT_TRUE(fs::path(include).is_absolute());
  EXPECT_TRUE(fs::exists(include));
  EXPECT_EQ(first, filter.prepare(source.string()));
  write("nested/leaf.xml", "<root><Other optional=\"\"/></root>");
  EXPECT_NE(first, filter.prepare(source.string()));
  EXPECT_NE(read(child).find("leaf.xml"), std::string::npos);
  EXPECT_EQ(read(leaf), "<root><Other optional=\"\"/></root>");
}
TEST_F(XmlFilter, RejectsMalformedMissingOrCyclicIncludesWithoutModifyingSource)
{
  auto malformed = write("bad.xml", "<root>");
  EXPECT_THROW(filter.prepare(malformed.string()), std::runtime_error);
  EXPECT_THROW(filter.prepare((directory / "missing.xml").string()), fs::filesystem_error);
  auto source = write("tree.xml", "<root><include path=\"tree.xml\"/></root>");
  EXPECT_THROW(filter.prepare(source.string()), std::runtime_error);
  write("tree.xml", "<root><include path=\"\"/></root>");
  EXPECT_THROW(filter.prepare(source.string()), std::runtime_error);
  write("tree.xml", "<root/>");
  EXPECT_NO_THROW(filter.prepare(source.string()));
}
TEST_F(XmlFilter, DeletesOnlyItsRuntimeDirectoryOnDestruction)
{
  auto source = write("tree.xml", "<root/>");
  fs::path runtime;
  {
    XmlTreeFilter temporary;
    runtime = fs::path(temporary.prepare(source.string())).parent_path();
    EXPECT_TRUE(fs::exists(runtime));
  }
  EXPECT_FALSE(fs::exists(runtime));
  EXPECT_TRUE(fs::exists(source));
}
