#include <map>
#include <set>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <proton_ros2_interfaces/adaptor_interface.hpp>
#include <yaml-cpp/yaml.h>

TEST(PlatformAdaptors, InstalledPluginsMatchOnlyTheirOwnerSchema)
{
  pluginlib::ClassLoader<proton_ros2_interfaces::MessageAdaptorIface> loader(
    "proton_ros2_interfaces", "proton_ros2_interfaces::MessageAdaptorIface");
  const std::map<std::string, std::string> schemas = {
    {"core", "core.yaml"},
    {"dingo", "d1x0.yaml"},
    {"jackal", "j100.yaml"},
    {"ridgeback", "r100.yaml"},
    {"warthog", "w200.yaml"},
  };
  const auto schema_dir =
    ament_index_cpp::get_package_share_directory("clearpath_firmware") + "/proton/";
  const auto classes = loader.getDeclaredClasses();
  ASSERT_FALSE(classes.empty());
  for (const auto & [platform, schema_file] : schemas) {
    const std::string package = "clearpath_proton_" + platform + "_adaptors";
    SCOPED_TRACE(package);
    const auto schema = YAML::LoadFile(schema_dir + schema_file);
    ASSERT_EQ(schema["adaptor_packages"].size(), 1u);
    ASSERT_EQ(schema["adaptor_packages"][0].as<std::string>(), package);
    std::set<std::string> expected;
    for (const auto & message : schema["messages"]) {
      expected.insert(message["name"].as<std::string>());
    }
    ASSERT_FALSE(expected.empty());
    std::set<std::string> loaded;
    for (const auto & class_id : classes) {
      ASSERT_NE(loader.getClassPackage(class_id), "clearpath_proton_adaptors");
      if (loader.getClassPackage(class_id) != package) {
        continue;
      }
      auto adaptor = loader.createSharedInstance(class_id);
      const auto binding = adaptor->get_binding_name();
      EXPECT_EQ(class_id, "proton_ros2_adaptors::" + binding + "Adaptor");
      EXPECT_TRUE(loaded.insert(binding).second);
      EXPECT_FALSE(adaptor->get_message_type().empty());
    }
    EXPECT_EQ(loaded, expected);
  }
}
