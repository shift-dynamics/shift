/* the records shaders share must have the same layout in each. The
instances (RtInstance in viewport.cc) are read by raytrace.comp and by the
denoiser's temporal and TAA passes, and a field
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
  const std::regex field("(\\w+)\\s+(\\w+(?:\\[\\d+\\])?)\\s*;");
  for(std::sregex_iterator it(body.begin(), body.end(), field), last;
      it != last; ++it) {
    fields.push_back((*it)[1].str() + " " + (*it)[2].str());
  }
  return fields;
}

TEST(ShaderLayouts, InstancesMatchTheRayTracer) {
  const auto fields = Fields("raytrace.comp", "Instance");
  ASSERT_FALSE(fields.empty());
  /* mat4 x 3 (48 floats), vec4 x 8, ivec4 x 4 and vec4[2] (56): 416 bytes,
  the size of RtInstance */
  EXPECT_EQ(fields.size(), 16u);
  for(const char* shader :
      {"svgf_temporal.comp", "svgf_taa.comp", "dlss_prepare.comp"}) {
    EXPECT_EQ(Fields(shader, "Instance"), fields) << shader;
  }
}

/* the material of a mesh (MaterialUniforms in viewport.cc) is declared by
both stages of the mesh pipeline */
TEST(ShaderLayouts, MaterialsMatchInBothStages) {
  auto block = [](const std::string& shader) {
    std::ifstream in(std::string(SHIFT_ROOT_DIR) + "/shift/gui/shaders/" +
                     shader);
    std::stringstream text;
    text << in.rdbuf();
    const std::string source =
      std::regex_replace(text.str(), std::regex("//[^\n]*"), "");
    const size_t start = source.find("uniform Material {");
    EXPECT_NE(start, std::string::npos) << shader;
    const std::string body =
      source.substr(start, source.find("}", start) - start);
    std::vector<std::string> fields;
    const std::regex field("(\\w+)\\s+(\\w+(?:\\[\\d+\\])?)\\s*;");
    for(std::sregex_iterator it(body.begin(), body.end(), field), last;
        it != last; ++it) {
      fields.push_back((*it)[1].str() + " " + (*it)[2].str());
    }
    return fields;
  };
  const auto fields = block("mesh.frag");
  /* vec4 x 5, ivec4 x 2, vec4[2] and vec4[4]: 208 bytes, MaterialUniforms */
  EXPECT_EQ(fields.size(), 9u);
  EXPECT_EQ(block("mesh.vert"), fields);
  EXPECT_EQ(block("particle.vert"), fields);
}

/* the fields of the uniform block name in a shader, without comments */
static std::vector<std::string> Block(const std::string& shader,
                                      const std::string& name) {
  std::ifstream in(std::string(SHIFT_ROOT_DIR) + "/shift/gui/shaders/" +
                   shader);
  std::stringstream text;
  text << in.rdbuf();
  const std::string source =
    std::regex_replace(text.str(), std::regex("//[^\n]*"), "");
  const size_t start = source.find("uniform " + name + " {");
  if(start == std::string::npos) {
    ADD_FAILURE() << shader << " has no uniform block " << name;
    return {};
  }
  const std::string body = source.substr(start, source.find("}", start) - start);
  std::vector<std::string> fields;
  const std::regex field("(\\w+)\\s+(\\w+(?:\\[\\d+\\])?)\\s*;");
  for(std::sregex_iterator it(body.begin(), body.end(), field), last;
      it != last; ++it) {
    fields.push_back((*it)[1].str() + " " + (*it)[2].str());
  }
  return fields;
}

/* the frame (FrameUniforms in viewport.cc) as the fragment shaders that
draw the water over what they draw declare it: all of it up to the water's
parameters; after them only mesh.frag declares the colors of the stress
drawn on flexible bodies */
TEST(ShaderLayouts, FramesMatchInTheFragmentShaders) {
  const auto all = Block("mesh.frag", "Frame");
  /* mat4 x 2, vec4 x 6, mat4, vec4 x 6, vec4[16] */
  ASSERT_EQ(all.size(), 16u);
  EXPECT_EQ(all.back(), "vec4 color_map[16]");
  const std::vector<std::string> fields(all.begin(), all.end() - 1);
  EXPECT_EQ(fields.back(), "vec4 water_extinction");
  for(const char* shader : {"floor.frag", "sky.frag"}) {
    EXPECT_EQ(Block(shader, "Frame"), fields) << shader;
  }
}

/* the material maps of the terrains (GroundBuffer in viewport.cc): their
records in ground.glsl, 96 and 48 bytes */
TEST(ShaderLayouts, GroundRecords) {
  const std::vector<std::string> map = {"mat4 world_to_map", "vec4 scale",
                                        "ivec4 size"};
  const std::vector<std::string> material = {"vec4 color", "vec4 scale",
                                             "ivec4 layers"};
  EXPECT_EQ(Fields("ground.glsl", "GroundMap"), map);
  EXPECT_EQ(Fields("ground.glsl", "GroundMaterial"), material);
}
