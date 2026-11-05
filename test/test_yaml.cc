#include <gtest/gtest.h>

#include <json/json.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/parameter/yaml.h"

static std::string ReadFile(const std::string& path) {
  std::ifstream file(path);
  std::stringstream text;
  text << file.rdbuf();
  return text.str();
}

static Json::Value ReadJson(const std::string& path) {
  Json::Value root;
  std::ifstream file(path);
  file >> root;
  return root;
}

TEST(Yaml, TheOrganizationOfAModelIsTheSameAsInJson) {
  const std::string dir = std::string(SHIFT_EXAMPLES_DIR) + "/pendulum/";
  EXPECT_EQ(shift::core::parameter::YamlToJson(ReadFile(dir + "pendulum.yaml")),
            ReadJson(dir + "pendulum.json"));
}

TEST(Yaml, PlainScalarsHaveTheTypesOfJson) {
  const Json::Value root = shift::core::parameter::YamlToJson(
    "integer: 12\n"
    "negative: -3\n"
    "hex: 0x1F\n"
    "real: 1.5\n"
    "whole_real: 2.0\n"
    "exponent: 1e-3\n"
    "leading_dot: .5\n"
    "boolean: true\n"
    "off: false\n"
    "nothing: null\n"
    "tilde: ~\n"
    "empty:\n"
    "text: revolute\n"
    "version: 0.2.0\n"
    "yes_is_text: yes\n"
    "infinity: .inf\n"
    "not_a_number: .nan\n"
    "big: 18446744073709551615\n"
    "quoted_number: \"12\"\n"
    "quoted_bool: 'true'\n"
    "list: [1, 2.0, three]\n");
  EXPECT_EQ(root["integer"], Json::Value(12));
  EXPECT_TRUE(root["integer"].isInt());
  EXPECT_EQ(root["negative"], Json::Value(-3));
  EXPECT_EQ(root["hex"], Json::Value(31));
  EXPECT_TRUE(root["real"].isDouble());
  EXPECT_EQ(root["real"].asDouble(), 1.5);
  EXPECT_TRUE(root["whole_real"].isDouble());
  EXPECT_EQ(root["whole_real"].asDouble(), 2.0);
  EXPECT_EQ(root["exponent"].asDouble(), 1e-3);
  EXPECT_EQ(root["leading_dot"].asDouble(), 0.5);
  EXPECT_TRUE(root["boolean"].isBool());
  EXPECT_TRUE(root["boolean"].asBool());
  EXPECT_FALSE(root["off"].asBool());
  EXPECT_TRUE(root["nothing"].isNull());
  EXPECT_TRUE(root["tilde"].isNull());
  EXPECT_TRUE(root["empty"].isNull());
  EXPECT_EQ(root["text"], Json::Value("revolute"));
  EXPECT_EQ(root["version"], Json::Value("0.2.0"));
  EXPECT_EQ(root["yes_is_text"], Json::Value("yes"));
  EXPECT_TRUE(std::isinf(root["infinity"].asDouble()));
  EXPECT_TRUE(std::isnan(root["not_a_number"].asDouble()));
  EXPECT_TRUE(root["big"].isUInt64());
  EXPECT_EQ(root["big"].asUInt64(), 18446744073709551615ull);
  EXPECT_EQ(root["quoted_number"], Json::Value("12"));
  EXPECT_EQ(root["quoted_bool"], Json::Value("true"));
  ASSERT_EQ(root["list"].size(), 3u);
  EXPECT_TRUE(root["list"][0].isInt());
  EXPECT_TRUE(root["list"][1].isDouble());
  EXPECT_EQ(root["list"][2], Json::Value("three"));
}

TEST(Yaml, AnchorsAliasesAndMergeKeysAreResolved) {
  const Json::Value root = shift::core::parameter::YamlToJson(
    "# a comment\n"
    "defaults: &defaults\n"
    "  radius: 0.05\n"
    "  density: 7800\n"
    "other: &other\n"
    "  color: red\n"
    "a:\n"
    "  <<: *defaults\n"
    "  radius: 0.1\n"
    "b:\n"
    "  <<: [*defaults, *other]\n"
    "same: *other\n");
  EXPECT_EQ(root["a"]["radius"].asDouble(), 0.1);  // its own entry wins
  EXPECT_EQ(root["a"]["density"].asInt(), 7800);
  EXPECT_EQ(root["b"]["radius"].asDouble(), 0.05);
  EXPECT_EQ(root["b"]["color"].asString(), "red");
  EXPECT_EQ(root["same"], root["other"]);
  EXPECT_FALSE(root["a"].isMember("<<"));
}

TEST(Yaml, ABadDocumentIsAnErrorWithItsLine) {
  try {
    shift::core::parameter::YamlToJson("a: 1\nb: [1, 2\nc: 3\n");
    FAIL() << "expected an exception";
  } catch(const shift::core::Exception& error) {
    EXPECT_NE(std::string(error.what()).find("line"), std::string::npos);
  }
  EXPECT_THROW(shift::core::parameter::YamlToJson("? [1, 2]\n: x\n"),
               shift::core::Exception);
  EXPECT_THROW(shift::core::parameter::YamlToJson("a: &x\n  b: *x\n"),
               shift::core::Exception);
}

TEST(Yaml, TheLoaderChoosesTheLanguageByTheExtension) {
  const std::string dir =
    (std::filesystem::path(SHIFT_BINARY_DIR) / "test" / "yaml").string();
  std::filesystem::create_directories(dir);
  const std::string source =
    std::string(SHIFT_EXAMPLES_DIR) + "/pendulum/pendulum.yaml";
  for(const char* name : {"model.YML", "model.yaml"}) {
    // the model's include directory is the one of the example
    std::string text = ReadFile(source);
    const std::string old_value = "include_directories: [.]";
    const size_t at = text.find(old_value);
    ASSERT_NE(at, std::string::npos);
    text.replace(at, old_value.size(),
                 "include_directories: [" + std::string(SHIFT_EXAMPLES_DIR) +
                   "/pendulum]");
    std::ofstream(dir + "/" + name) << text;
    shift::core::Context context;
    EXPECT_NO_THROW(context.LoadJSON(dir + "/" + name)) << name;
  }
  // a document that is not a mapping is not a model
  std::ofstream(dir + "/list.yaml") << "- 1\n- 2\n";
  shift::core::Context context;
  EXPECT_THROW(context.LoadJSON(dir + "/list.yaml"), shift::core::Exception);
}
