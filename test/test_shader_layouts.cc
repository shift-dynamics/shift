/* the records the ray tracer's compute shaders share must have the same
layout in each shader: the instances (RtInstance in viewport.cc) are read by
raytrace.comp and by the denoiser's temporal and TAA passes, and a field
missing from one of them shifts every instance after the first, so that the
denoiser reprojects the bodies through the poses of others */

#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "shift/config.h"

/* the fields (type and name) of struct name in a shader, without comments */
static std::vector<std::string> Fields(const std::string& shader,
                                       const std::string& name) {
  std::ifstream in(std::string(SHIFT_ROOT_DIR) + "/shift/gui/shaders/" +
                   shader);
  EXPECT_TRUE(in.good()) << shader;
  std::stringstream text;
  text << in.rdbuf();
  const std::string source =
    std::regex_replace(text.str(), std::regex("//[^\n]*"), "");
  const size_t start = source.find("struct " + name + " {");
  if(start == std::string::npos) {
    ADD_FAILURE() << shader << " has no struct " << name;
    return {};
  }
  const size_t end = source.find("};", start);
  const std::string body =
    source.substr(start, end - start).substr(name.size() + 9);
  std::vector<std::string> fields;
  const std::regex field("(\\w+)\\s+(\\w+)\\s*;");
  for(std::sregex_iterator it(body.begin(), body.end(), field), last;
      it != last; ++it) {
    fields.push_back((*it)[1].str() + " " + (*it)[2].str());
  }
  return fields;
}

TEST(ShaderLayouts, InstancesMatchTheRayTracer) {
  const auto fields = Fields("raytrace.comp", "Instance");
  ASSERT_FALSE(fields.empty());
  /* mat4 x 3 (48 floats), vec4 x 5 and ivec4 x 1 (24): 288 bytes, the size
  of RtInstance */
  EXPECT_EQ(fields.size(), 9u);
  for(const char* shader : {"svgf_temporal.comp", "svgf_taa.comp"}) {
    EXPECT_EQ(Fields(shader, "Instance"), fields) << shader;
  }
}
