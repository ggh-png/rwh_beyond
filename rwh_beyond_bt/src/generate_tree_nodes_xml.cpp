#include <fstream>
#include <iostream>
#include <stdexcept>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/xml_parsing.h"

int main(int argc, char ** argv)
{
  if (argc > 2) {
    std::cerr << "Usage: generate_tree_nodes_xml [output.xml]\n";
    return 1;
  }
  try {
    BT::BehaviorTreeFactory factory;
    factory.registerFromPlugin(BT::SharedLibrary::getOSName("rwh_beyond_bt_nodes"));
    const std::string path = argc == 2 ? argv[1] : "rwh_beyond_tree_nodes.xml";
    std::ofstream output;
    output.exceptions(std::ios::failbit | std::ios::badbit);
    output.open(path);
    output << BT::writeTreeNodesModelXML(factory) << std::endl;
    output.close();
    std::cout << "Generated " << path << std::endl;
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    return 1;
  }
  return 0;
}
