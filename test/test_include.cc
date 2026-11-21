#include <gtest/gtest.h>

#include <json/json.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <set>
#include <string>

#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/parameter/include.h"
#include "shift/core/parameter/model_writer.h"
#include "shift/core/light/base.h"
#include "shift/gui/simulation.h"

/* model files in different directories that include one another:

  root/models/model.json            the model, written by each test
  root/left/arm.json, root/right/arm.json
                                    the same arm, each with its own
  root/left/meshes/link.obj, root/right/meshes/link.obj
                                    found by its arm's include directory
  root/environments/flat/flat.json  a frame, a sky and a logger
  root/settings/solver.json         model_parameters that include another
*/
class IncludeTest : public testing::Test {
protected:
  void SetUp() override {
    root_ = std::filesystem::path(testing::TempDir()) / "shift_include";
    std::filesystem::remove_all(root_);
    for(const char* directory :
        {"models", "left/meshes", "right/meshes", "environments/flat",
         "settings"}) {
      std::filesystem::create_directories(root_ / directory);
    }
    const std::string pendulum = std::string(SHIFT_EXAMPLES_DIR) + "/pendulum/";
    for(const char* side : {"left", "right"}) {
      for(const char* file : {"link.obj", "link.mtl"}) {
        std::filesystem::copy_file(pendulum + file,
                                   root_ / side / "meshes" / file);
      }
      Write(std::string(side) + "/arm.json", R"({
        // an arm; its mesh is found in its own include directory
        "model_parameters": {"include_directories": ["meshes"],
                             "gravity": [0.0, 0.0, 1.0],
                             "epsilon_pivot_ratio": 0.5,
                             "solvers": [{"type": "rnca"}]},
        "assemblies": [{
          "name": "arm",
          "bodies": [{
            "name": "link1",
            "center_of_mass": [0.0, 1.5, 0.0],
            "inertia": {"mass": 1.0, "Ixx": 1.0, "Iyy": 1.0, "Izz": 1.0,
                        "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0},
            "geometry": [{"name": "link", "type": "mesh",
                          "filename": ["link.obj"]}]}],
          "joints": [{
            "name": "fixed_to_link1", "type": "revolute",
            "parent": [["fixed"], "origin"], "child": [["link1"], "origin"],
            "initial_displacement": 1.0, "axis": [1, 0, 0]}]}]
      })");
    }
    Write("environments/flat/flat.json", R"({
      "model_parameters": {"loggers": [{"type": "csv",
                                        "csv_logfile": "flat.csv"}]},
      "frames": [{"name": "flat", "T": [1.0, 2.0, 0.0]}],
      "environment": {"sky": {"type": "physical", "turbidity": 3.0},
                      "sun": {"azimuth": 140.0, "elevation": 35.0}}
    })");
    Write("settings/solver.json", R"({
      "model_parameters": {"include": "../left/arm.json"}
    })");
  }

  void TearDown() override {
    std::filesystem::remove_all(root_);
  }

  void Write(const std::string& file, const std::string& text) {
    std::ofstream(root_ / file) << text;
  }

  std::string Model(const std::string& text) {
    Write("models/model.json", text);
    return (root_ / "models/model.json").string();
  }

  std::string Path(const std::string& file) {
    return (root_ / file).lexically_normal().string();
  }

  std::filesystem::path root_;
};

/* the model's own parameters */
static std::string Parameters() {
  return R"(
  "model_parameters": {
    "gravity": [0.0, 0.0, 9.81],
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": 0.1, "time_step": 0.01}]
  })";
}

TEST_F(IncludeTest, AnIncludedModelsAssembliesAreSubassemblies) {
  shift::core::parameter::Origins origins;
  const Json::Value model =
    shift::core::parameter::Include::ReadFile(Model("{" + Parameters() + R"(,
      "frames": [{"name": "start", "T": [0.0, 0.0, 0.0]}],
      "environment": {"sun": {"elevation": 10.0}},
      "assemblies": [
        {"name": "left", "include": "../left/arm.json",
         "assemblies": [{"name": "own"}]},
        {"name": "right",
         "include": ["../right/arm.json", "../environments/flat/flat.json"]}]
    })"), nullptr, &origins);
  // each arm in its own assembly, after the subassemblies listed
  const Json::Value& left = model["assemblies"][0];
  ASSERT_EQ(left["assemblies"].size(), 2u);
  EXPECT_EQ(left["assemblies"][0]["name"].asString(), "own");
  EXPECT_EQ(left["assemblies"][1]["name"].asString(), "arm");
  EXPECT_EQ(left["assemblies"][1]["bodies"].size(), 1u);
  EXPECT_FALSE(left.isMember("include"));
  EXPECT_FALSE(left.isMember("bodies"));
  const Json::Value& right = model["assemblies"][1];
  ASSERT_EQ(right["assemblies"].size(), 1u);
  EXPECT_EQ(right["assemblies"][0]["name"].asString(), "arm");
  // the rest of an included top level goes to the model's: arrays of
  // objects joined, the model's first; a vector is never joined
  ASSERT_EQ(model["frames"].size(), 2u);
  EXPECT_EQ(model["frames"][0]["name"].asString(), "start");
  EXPECT_EQ(model["frames"][1]["name"].asString(), "flat");
  EXPECT_EQ(model["frames"][1]["T"].size(), 3u);
  EXPECT_EQ(model["environment"]["sky"]["type"].asString(), "physical");
  EXPECT_DOUBLE_EQ(model["environment"]["sun"]["azimuth"].asDouble(), 140.0);
  EXPECT_DOUBLE_EQ(model["environment"]["sun"]["elevation"].asDouble(), 10.0);
  // the arms' parameters folded in: the model's win where both give one,
  // the solvers and integrators are the model's whole (each one listed
  // runs), and a key only an arm gives is taken
  const Json::Value& parameters = model["model_parameters"];
  EXPECT_DOUBLE_EQ(parameters["epsilon_pivot_ratio"].asDouble(), 0.5);
  EXPECT_EQ(parameters["integrators"][0]["type"].asString(), "ode113");
  EXPECT_DOUBLE_EQ(parameters["gravity"][2].asDouble(), 9.81);
  EXPECT_EQ(parameters["gravity"].size(), 3u);
  ASSERT_EQ(parameters["solvers"].size(), 1u);
  EXPECT_EQ(parameters["solvers"][0]["type"].asString(), "rnca");
  EXPECT_EQ(parameters["integrators"].size(), 1u);
  EXPECT_EQ(parameters["loggers"].size(), 1u);
  // an arm's include directories are its own, not the model's
  EXPECT_FALSE(parameters.isMember("include_directories"));
  // where each part from an included file comes from, kept beside the
  // model, which holds only what the files hold
  const shift::core::parameter::Origin* arm =
    origins.At("/assemblies/0/assemblies/1");
  ASSERT_NE(arm, nullptr);
  EXPECT_EQ(arm->file, Path("left/arm.json"));
  Json::Value path(Json::arrayValue);
  path.append("assemblies");
  path.append(Json::ArrayIndex(0));
  EXPECT_EQ(arm->path, path);
  ASSERT_EQ(arm->include_directories.size(), 1u);
  EXPECT_EQ(arm->include_directories[0], Path("left/meshes"));
  ASSERT_NE(origins.At("/frames/1"), nullptr);
  EXPECT_EQ(origins.At("/frames/1")->file,
            Path("environments/flat/flat.json"));
  EXPECT_EQ(origins.At("/frames/0"), nullptr);
  EXPECT_EQ(origins.At("/assemblies/0/assemblies/0"), nullptr);
  // the sky came from flat.json into the model's environment
  ASSERT_NE(origins.At("/environment/sky"), nullptr);
  EXPECT_EQ(origins.At("/environment/sky")->file,
            Path("environments/flat/flat.json"));
  EXPECT_EQ(origins.At("/environment"), nullptr);
  Json::StreamWriterBuilder writer;
  EXPECT_EQ(Json::writeString(writer, model).find("$source"),
            std::string::npos);
}

TEST_F(IncludeTest, AWarningNamesEachParameterTheModelOverrides) {
  testing::internal::CaptureStderr();
  shift::core::parameter::Include::ReadFile(Model("{" + Parameters() + R"(,
      "assemblies": [{"name": "left", "include": "../left/arm.json"},
                     {"name": "right", "include": "../right/arm.json"}]
    })"));
  const std::string warnings = testing::internal::GetCapturedStderr();
  // each arm's gravity differs from the model's
  for(const char* arm : {"left/arm.json", "right/arm.json"}) {
    EXPECT_NE(warnings.find("gravity of '" + Path(arm) + "' ([0.0,0.0,1.0])"),
              std::string::npos)
      << warnings;
  }
  // not a value that agrees (the right arm's epsilon_pivot_ratio, as the
  // left's; the arms' solver, as the model's)
  EXPECT_EQ(warnings.find("epsilon_pivot_ratio"), std::string::npos)
    << warnings;
  EXPECT_EQ(warnings.find("solvers"), std::string::npos) << warnings;
}

TEST_F(IncludeTest, IncludeIsAllowedOnlyInAnAssembly) {
  Write("left/loop.json",
        R"({"assemblies": [{"name": "loop", "include": "loop.json"}]})");
  for(const std::string& text :
      {std::string(R"({"include": "../left/arm.json"})"),
       std::string(R"({"terrain": [{"include": "../left/arm.json"}]})"),
       std::string(R"({"assemblies": [{"name": "a", "bodies": [
                     {"include": "../left/arm.json"}]}]})"),
       std::string(R"({"assemblies": [{"name": "a",
                     "include": "../left/missing.json"}]})"),
       std::string(R"({"assemblies": [{"name": "a",
                     "include": "../left/loop.json"}]})"),
       std::string(R"({"assemblies": [{"name": "a", "include": 3}]})"),
       std::string(R"({"model_parameters": {
                     "include": "../left/arm.json"}})"),
       // nor in the model_parameters of an included file
       std::string(R"({"assemblies": [{"name": "a",
                     "include": "../settings/solver.json"}]})")}) {
    const std::string file = Model(text);
    EXPECT_THROW(shift::core::parameter::Include::ReadFile(file),
                 shift::core::Exception)
      << text;
  }
}

TEST_F(IncludeTest, EachIncludedFileFindsItsOwnFiles) {
  shift::core::Context context;
  context.LoadJSON(Model("{" + Parameters() + R"(,
    "assemblies": [{"name": "left", "include": "../left/arm.json"},
                   {"name": "right", "include": "../right/arm.json"}]
  })"));
  // the same name, link.obj, found by each arm in its own directory
  const auto& read = context.get_files_read();
  for(const char* file :
      {"left/meshes/link.obj", "right/meshes/link.obj", "left/arm.json",
       "right/arm.json"}) {
    EXPECT_NE(std::find(read.begin(), read.end(), Path(file)), read.end())
      << file;
  }
  // a name the model file gives itself is not found there
  EXPECT_TRUE(context.FindFile("link.obj").empty());
  // the model reloads as it was read (as the viewer does)
  shift::core::Context again;
  again.LoadJSON(context.get_model_json(), context.get_filename(),
                 &context.get_origins());
  EXPECT_EQ(again.get_model_json(), context.get_model_json());
  EXPECT_EQ(again.get_files_read(), context.get_files_read());
}

TEST_F(IncludeTest, ALogsFilesAreFoundByThePathsTheyWereFoundAt) {
  shift::core::parameter::List::Source source;
  source.file = Path("left/arm.json");
  source.include_directories = {Path("left/meshes")};
  shift::core::Context context;
  EXPECT_EQ(context.FindFile("link.obj", &source),
            Path("left/meshes/link.obj"));
  // as a log maps them, by the path found when the log was written
  context.set_file_map({{Path("left/meshes/link.obj"), "/log/1"},
                        {Path("right/meshes/link.obj"), "/log/2"},
                        {"link.obj", "/log/2"}});
  EXPECT_EQ(context.FindFile("link.obj", &source), "/log/1");
}

static std::string Text(const std::filesystem::path& file) {
  std::ifstream in(file);
  return std::string((std::istreambuf_iterator<char>(in)),
                     std::istreambuf_iterator<char>());
}

TEST_F(IncludeTest, SaveInPlaceWritesEachEditToTheFileItBelongsTo) {
  const std::string file = Model("{" + Parameters() + R"(,
    "assemblies": [{"name": "left", "include": "../left/arm.json"},
                   {"name": "right",
                    "include": ["../right/arm.json",
                                "../environments/flat/flat.json"]}]
  })");
  shift::core::parameter::Origins origins;
  const Json::Value loaded =
    shift::core::parameter::Include::ReadFile(file, nullptr, &origins);
  Json::Value edited = loaded;
  // in the left arm, a model parameter, and in the environment's sky and
  // frame, which came from flat.json into the model's top level
  edited["assemblies"][0]["assemblies"][0]["bodies"][0]["center_of_mass"][1] =
    2.0;
  edited["model_parameters"]["gravity"][2] = 9.8;
  edited["environment"]["sky"]["turbidity"] = 4.0;
  edited["frames"][0]["T"][0] = 5.0;
  const std::string right = Text(root_ / "right/arm.json");
  const std::vector<std::string> written =
    shift::core::parameter::ModelWriter::SaveInPlace(loaded, edited, file, origins);
  EXPECT_EQ(std::set<std::string>(written.begin(), written.end()),
            (std::set<std::string>{Path("models/model.json"),
                                   Path("left/arm.json"),
                                   Path("environments/flat/flat.json")}));
  // the files keep their includes, their comments and their own keys
  const Json::Value model =
    shift::core::parameter::Include::Parse(root_ / "models/model.json");
  EXPECT_EQ(model["assemblies"][0]["include"].asString(), "../left/arm.json");
  EXPECT_FALSE(model.isMember("environment"));
  EXPECT_DOUBLE_EQ(model["model_parameters"]["gravity"][2].asDouble(), 9.8);
  const Json::Value arm =
    shift::core::parameter::Include::Parse(root_ / "left/arm.json");
  EXPECT_DOUBLE_EQ(
    arm["assemblies"][0]["bodies"][0]["center_of_mass"][1].asDouble(), 2.0);
  EXPECT_EQ(arm["model_parameters"]["include_directories"][0].asString(),
            "meshes");
  EXPECT_NE(Text(root_ / "left/arm.json").find("// an arm"),
            std::string::npos);
  const Json::Value flat = shift::core::parameter::Include::Parse(
    root_ / "environments/flat/flat.json");
  EXPECT_DOUBLE_EQ(flat["environment"]["sky"]["turbidity"].asDouble(), 4.0);
  EXPECT_DOUBLE_EQ(flat["frames"][0]["T"][0].asDouble(), 5.0);
  EXPECT_EQ(Text(root_ / "right/arm.json"), right);
  for(const char* saved : {"models/model.json", "left/arm.json"}) {
    EXPECT_EQ(Text(root_ / saved).find("$source"), std::string::npos);
  }
  // read again, the model is the model edited
  EXPECT_EQ(shift::core::parameter::Include::ReadFile(file), edited);
  // and with no edits, nothing is written
  EXPECT_TRUE(
    shift::core::parameter::ModelWriter::SaveInPlace(edited, edited, file, origins)
      .empty());
}

TEST_F(IncludeTest, SaveFlattenedCopiesTheFilesOfEachIncludeApart) {
  shift::core::Context context;
  context.LoadJSON(Model("{" + Parameters() + R"(,
    "assemblies": [{"name": "left", "include": "../left/arm.json"},
                   {"name": "right", "include": "../right/arm.json"}]
  })"));
  const std::string output = Path("out/flat_model.json");
  std::filesystem::create_directories(root_ / "out");
  shift::core::parameter::ModelWriter::SaveFlattened(
    context.get_model_json(), context.get_filename(), context.get_origins(),
    context.get_files_found(), context.get_files_read(), output);
  const std::string text = Text(output);
  EXPECT_EQ(text.find("\"include\""), std::string::npos);
  EXPECT_EQ(text.find("$source"), std::string::npos);
  const Json::Value model =
    shift::core::parameter::Include::Parse(output);
  EXPECT_FALSE(model["model_parameters"].isMember("include_directories"));
  // each arm's link.obj, and the .mtl beside it, copied apart
  for(int side = 0; side < 2; side++) {
    const std::string name = side == 0 ? "left" : "right";
    const Json::Value& arm = model["assemblies"][side]["assemblies"][0];
    EXPECT_EQ(arm["bodies"][0]["geometry"][0]["filename"][0].asString(),
              "flat_model_files/" + name + "/meshes/link.obj");
    for(const char* file : {"link.obj", "link.mtl"}) {
      EXPECT_TRUE(std::filesystem::exists(
        root_ / "out/flat_model_files" / name / "meshes" / file))
        << name << "/" << file;
    }
  }
  // the flattened model loads, from the copies
  shift::core::Context flat;
  flat.LoadJSON(output);
  const auto& read = flat.get_files_read();
  EXPECT_NE(std::find(read.begin(), read.end(),
                      Path("out/flat_model_files/right/meshes/link.obj")),
            read.end());
}

TEST_F(IncludeTest, AnIncludedYamlFileIsSavedAsYaml) {
  Write("left/arm.yaml",
        "# an arm, in YAML\n"
        "model_parameters:\n"
        "  include_directories: [meshes]\n"
        "assemblies:\n"
        "  - name: arm\n"
        "    bodies:\n"
        "      - name: link1\n"
        "        center_of_mass: [0.0, 1.5, 0.0]\n"
        "        inertia: {mass: 1.0, Ixx: 1.0, Iyy: 1.0, Izz: 1.0,\n"
        "                  Ixy: 0.0, Iyz: 0.0, Izx: 0.0}\n"
        "        geometry:\n"
        "          - {name: link, type: mesh, filename: [link.obj]}\n"
        "    joints:\n"
        "      - name: fixed_to_link1\n"
        "        type: revolute\n"
        "        parent: [[fixed], origin]\n"
        "        child: [[link1], origin]\n"
        "        axis: [1, 0, 0]\n");
  const std::string file = Model("{" + Parameters() + R"(,
    "assemblies": [{"name": "left", "include": "../left/arm.yaml"}]
  })");
  shift::core::parameter::Origins origins;
  const Json::Value loaded =
    shift::core::parameter::Include::ReadFile(file, nullptr, &origins);
  Json::Value edited = loaded;
  edited["assemblies"][0]["assemblies"][0]["bodies"][0]["center_of_mass"][1] =
    2.0;
  EXPECT_EQ(
    shift::core::parameter::ModelWriter::SaveInPlace(loaded, edited, file, origins),
    std::vector<std::string>{Path("left/arm.yaml")});
  const std::string text = Text(root_ / "left/arm.yaml");
  EXPECT_EQ(text.rfind("# an arm, in YAML\nmodel_parameters:", 0), 0u) << text;
  EXPECT_NE(text.find("center_of_mass: [0.0, 2.0, 0.0]"), std::string::npos)
    << text;
  EXPECT_EQ(text.find("$source"), std::string::npos) << text;
  EXPECT_EQ(shift::core::parameter::Include::ReadFile(file), edited);

  // flattened to a model file in YAML
  shift::core::Context context;
  context.LoadJSON(file);
  std::filesystem::create_directories(root_ / "out");
  const std::string output = Path("out/flat_model.yaml");
  shift::core::parameter::ModelWriter::SaveFlattened(
    context.get_model_json(), context.get_filename(), context.get_origins(),
    context.get_files_found(), context.get_files_read(), output);
  const Json::Value flat = shift::core::parameter::Include::Parse(output);
  EXPECT_DOUBLE_EQ(flat["assemblies"][0]["assemblies"][0]["bodies"][0]
                       ["center_of_mass"][1]
                         .asDouble(),
                   2.0);
  EXPECT_FALSE(flat["assemblies"][0].isMember("include"));
  shift::core::Context reloaded;
  EXPECT_NO_THROW(reloaded.LoadJSON(output));
}

/* an arm hung from a frame at the top level of its file, with a lamp on
another; RK4, so that two models take the same steps */
static const char* kLampArm = R"({
  "model_parameters": {
    "gravity": [0.0, 0.0, 9.81],
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "rk4", "time_span": 0.2, "time_step": 0.01}]},
  "frames": [{"name": "pivot", "T": [0.0, 0.0, -2.0]},
             {"name": "lamp", "T": [0.0, 0.0, -5.0]}],
  "lights": [{"name": "lamp", "type": "point", "parent": ["fixed", "lamp"]}],
  "assemblies": [{
    "name": "arm",
    "bodies": [{"name": "link1", "center_of_mass": [0.0, 1.5, 0.0],
                "inertia": {"mass": 1.0, "Ixx": 1.0, "Iyy": 1.0, "Izz": 1.0,
                            "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0}}],
    "joints": [{"name": "hinge", "type": "revolute",
                "parent": [["fixed"], "pivot"], "child": [["link1"], "origin"],
                "initial_displacement": 1.0, "axis": [1, 0, 0]}]}]
})";

/* the arm's hinge and link of assembly in context, after it ran */
static void ExpectSameArm(shift::core::Context& placed,
                          shift::core::Context& reference,
                          const std::string& assembly) {
  shift::core::body::Base* a = placed.FindActiveBody(assembly + "__arm__link1");
  shift::core::body::Base* b =
    reference.FindActiveBody(assembly + "__arm__link1");
  ASSERT_NE(a, nullptr) << assembly;
  ASSERT_NE(b, nullptr) << assembly;
  const ksl_SE3_t pa = placed.get_body_origin_pose(a);
  const ksl_SE3_t pb = reference.get_body_origin_pose(b);
  for(int k = 0; k < 12; k++) {
    EXPECT_NEAR(pa.as_array[k], pb.as_array[k], 1e-12) << assembly << " " << k;
  }
}

/* the include of the lamp arm as the copy name, placed by overriding the
frames of the file, their names, and the names that refer to them */
static std::string LampArmCopy(const std::string& name,
                               const std::string& pivot_pose) {
  return R"({"name": ")" + name + R"(", "include": {
    "file": "../lamp/lamp_arm.json",
    "override": {
      "frames": [{"name": ")" + name + R"(_pivot", )" + pivot_pose + R"(},
                 {"name": ")" + name + R"(_lamp"}],
      "lights": [{"name": ")" + name + R"(_lamp",
                  "parent": ["fixed", ")" + name + R"(_lamp"]}],
      "assemblies": [{"joints": [{"parent": [["fixed"], ")" + name +
         R"(_pivot"]}]}]}}})";
}

TEST_F(IncludeTest, AFileIncludedTwiceIsPlacedByItsOverrides) {
  std::filesystem::create_directories(root_ / "lamp");
  Write("lamp/lamp_arm.json", kLampArm);
  // the arm twice, at two poses, the second turned a quarter about z (the
  // axis of gravity, so that each swings as the other)
  const std::string turn = R"("R": [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0],
                                    [0.0, 0.0, 1.0]])";
  shift::core::Context placed;
  placed.LoadJSON(Model(R"({
    "model_parameters": {
      "gravity": [0.0, 0.0, 9.81], "solvers": [{"type": "rnca"}],
      "integrators": [{"type": "rk4", "time_span": 0.2, "time_step": 0.01}]},
    "assemblies": [)" + LampArmCopy("a", R"("T": [0.0, -3.0, -2.0])") + ", " +
                       LampArmCopy("b", R"("T": [2.0, 3.0, -2.0], )" + turn) +
                       R"(]
  })"));
  // the same, built by hand: the pivots placed on the fixed body
  const std::string link = R"({"name": "link1",
    "center_of_mass": [0.0, 1.5, 0.0],
    "inertia": {"mass": 1.0, "Ixx": 1.0, "Iyy": 1.0, "Izz": 1.0,
                "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0}})";
  auto arm = [&](const std::string& name, const std::string& pivot) {
    return R"({"name": ")" + name + R"(", "assemblies": [{"name": "arm",
      "bodies": [)" + link + R"(],
      "joints": [{"name": "hinge", "type": "revolute",
                  "parent": [["fixed"], ")" + pivot + R"("],
                  "child": [["link1"], "origin"],
                  "initial_displacement": 1.0, "axis": [1, 0, 0]}]}]})";
  };
  Write("models/reference.json", R"({
    "model_parameters": {
      "gravity": [0.0, 0.0, 9.81], "solvers": [{"type": "rnca"}],
      "integrators": [{"type": "rk4", "time_span": 0.2, "time_step": 0.01}]},
    "frames": [{"name": "a_pivot", "T": [0.0, -3.0, -2.0]},
               {"name": "b_pivot", "T": [2.0, 3.0, -2.0], )" + turn + R"(}],
    "assemblies": [)" + arm("a", "a_pivot") + ", " + arm("b", "b_pivot") +
                                  R"(]
  })");
  shift::core::Context reference;
  reference.LoadJSON(Path("models/reference.json"));
  // one fixed body, with each copy's frames and lamp, named apart
  const Json::Value& model = placed.get_model_json();
  ASSERT_EQ(model["frames"].size(), 4u);
  EXPECT_EQ(model["frames"][2]["name"].asString(), "b_pivot");
  EXPECT_DOUBLE_EQ(model["frames"][2]["T"][1].asDouble(), 3.0);
  ASSERT_EQ(placed.get_active_lights().size(), 2u);
  for(const auto* light : placed.get_active_lights()) {
    ASSERT_NE(light->get_frame(), nullptr) << light->get_name();
    EXPECT_EQ(light->get_parent_body(), placed.get_fixed_body());
  }
  for(const char* assembly : {"a", "b"}) {
    ExpectSameArm(placed, reference, assembly);
  }
  // and each swings as the arm built by hand does
  auto run = [](shift::core::Context& context) {
    std::vector<double> y; /* at the last output */
    context.set_state_observer(
      [&](double, const std::vector<double>& state) { y = state; });
    context.Simulate();
    std::map<std::string, double> q;
    for(auto* joint : context.get_active_joints()) {
      q[joint->FullyQualifiedName(joint->get_name())] =
        y.at(joint->get_q_index());
    }
    return q;
  };
  std::map<std::string, double> placed_q = run(placed);
  std::map<std::string, double> reference_q = run(reference);
  for(const char* q : {"a__arm__hinge", "b__arm__hinge"}) {
    ASSERT_TRUE(placed_q.count(q)) << q;
    EXPECT_NE(placed_q[q], 1.0) << q; /* it swung */
    EXPECT_NEAR(placed_q[q], reference_q[q], 1e-12) << q;
  }
}

TEST_F(IncludeTest, AnOverrideIsLaidOutAsTheFile) {
  shift::core::parameter::Origins origins;
  const std::string file = Model("{" + Parameters() + R"(,
    "assemblies": [{"name": "left", "include": {
      "file": "../left/arm.json",
      "override": {
        "model_parameters": {"epsilon_pivot_ratio": 0.25},
        "assemblies": [{"bodies": [{"inertia": {"mass": 2.0}}],
                        "joints": [{"initial_displacement": 0.5,
                                    "axis": [0, 1, 0]}]}]}}}]
  })");
  const Json::Value model =
    shift::core::parameter::Include::ReadFile(file, nullptr, &origins);
  const Json::Value& arm = model["assemblies"][0]["assemblies"][0];
  // the values given, by position; the rest as the file has it
  EXPECT_DOUBLE_EQ(arm["bodies"][0]["inertia"]["mass"].asDouble(), 2.0);
  EXPECT_DOUBLE_EQ(arm["bodies"][0]["inertia"]["Ixx"].asDouble(), 1.0);
  EXPECT_DOUBLE_EQ(arm["joints"][0]["initial_displacement"].asDouble(), 0.5);
  EXPECT_EQ(arm["joints"][0]["axis"][1].asInt(), 1); /* replaced whole */
  EXPECT_EQ(arm["joints"][0]["axis"][0].asInt(), 0);
  EXPECT_EQ(arm["joints"][0]["name"].asString(), "fixed_to_link1");
  EXPECT_DOUBLE_EQ(model["model_parameters"]["epsilon_pivot_ratio"].asDouble(),
                   0.25);
  // each value given is the override's, written in the model file
  const shift::core::parameter::Origin* mass =
    origins.At("/assemblies/0/assemblies/0/bodies/0/inertia/mass");
  ASSERT_NE(mass, nullptr);
  EXPECT_EQ(mass->file, Path("models/model.json"));
  EXPECT_TRUE(mass->patch);
  Json::StreamWriterBuilder writer;
  writer["indentation"] = "";
  EXPECT_EQ(Json::writeString(writer, mass->path),
            R"(["assemblies",0,"include","override","assemblies",0,"bodies",)"
            R"(0,"inertia","mass"])");
  EXPECT_EQ(origins.At("/assemblies/0/assemblies/0/bodies/0/inertia/Ixx"),
            nullptr);
  // so an edit to it is saved to the override, and one beside it to the arm
  Json::Value edited = model;
  edited["assemblies"][0]["assemblies"][0]["bodies"][0]["inertia"]["mass"] =
    3.0;
  edited["assemblies"][0]["assemblies"][0]["bodies"][0]["inertia"]["Ixx"] =
    4.0;
  shift::core::parameter::ModelWriter::SaveInPlace(model, edited, file,
                                                   origins);
  const Json::Value saved =
    shift::core::parameter::Include::Parse(root_ / "models/model.json");
  EXPECT_DOUBLE_EQ(saved["assemblies"][0]["include"]["override"]["assemblies"]
                        [0]["bodies"][0]["inertia"]["mass"]
                          .asDouble(),
                   3.0);
  const Json::Value saved_arm =
    shift::core::parameter::Include::Parse(root_ / "left/arm.json");
  EXPECT_DOUBLE_EQ(
    saved_arm["assemblies"][0]["bodies"][0]["inertia"]["Ixx"].asDouble(), 4.0);
  EXPECT_DOUBLE_EQ(
    saved_arm["assemblies"][0]["bodies"][0]["inertia"]["mass"].asDouble(),
    1.0);
  EXPECT_EQ(shift::core::parameter::Include::ReadFile(file), edited);
}

TEST_F(IncludeTest, AnOverrideThatDoesNotFitTheFileIsAnError) {
  for(const char* override :
      {R"({"assemblies": [{}, {}]})",           /* more than the file has */
       R"({"assemblies": [{"bodies": {"name": "x"}}]})", /* not an array */
       R"({"assemblies": [{"name": {"x": 1}}]})",        /* not an object */
       R"({"assemblies": ["arm"]})",                     /* not an object */
       R"({"model_parameters": {"include_directories": ["."]}})"}) {
    const std::string file = Model("{" + Parameters() + R"(,
      "assemblies": [{"name": "left", "include": {
        "file": "../left/arm.json", "override": )" + std::string(override) +
                                   "}}]}");
    EXPECT_THROW(shift::core::parameter::Include::ReadFile(file),
                 shift::core::Exception)
      << override;
  }
  // nor a key an include does not take: a pose among them, which is
  // given by overriding the file's frames
  for(const char* key : {"offset", "T"}) {
    EXPECT_THROW(shift::core::parameter::Include::ReadFile(Model(
                   "{" + Parameters() + R"(,
      "assemblies": [{"name": "left", "include": {
        "file": "../left/arm.json", ")" + key + R"(": [1, 0, 0]}}]})")),
                 shift::core::Exception)
      << key;
  }
}

TEST_F(IncludeTest, APlacedCopySavesToItsOverrideAndFlattens) {
  std::filesystem::create_directories(root_ / "lamp");
  Write("lamp/lamp_arm.json", kLampArm);
  const std::string file = Model(R"({
    "model_parameters": {
      "gravity": [0.0, 0.0, 9.81], "solvers": [{"type": "rnca"}],
      "integrators": [{"type": "rk4", "time_span": 0.2, "time_step": 0.01}]},
    "assemblies": [)" + LampArmCopy("a", R"("T": [0.0, -3.0, -2.0])") + R"(]
  })");
  shift::core::Context context;
  context.LoadJSON(file);
  const Json::Value& model = context.get_model_json();
  // only the keys the files hold
  EXPECT_FALSE(model["assemblies"][0].isMember("ground"));
  EXPECT_FALSE(model["assemblies"][0]["assemblies"][0].isMember("ground"));
  // an edit to the pivot, which the override gives, goes to the override;
  // to the lamp's place, which the arm's file gives, to that file
  Json::Value edited = model;
  edited["frames"][0]["T"][1] = -4.0;
  edited["frames"][1]["T"][2] = -5.5;
  shift::core::parameter::ModelWriter::SaveInPlace(model, edited, file,
                                                   context.get_origins());
  const Json::Value saved = shift::core::parameter::Include::Parse(file);
  EXPECT_DOUBLE_EQ(
    saved["assemblies"][0]["include"]["override"]["frames"][0]["T"][1]
      .asDouble(),
    -4.0);
  const Json::Value lamp_arm =
    shift::core::parameter::Include::Parse(root_ / "lamp/lamp_arm.json");
  EXPECT_DOUBLE_EQ(lamp_arm["frames"][0]["T"][1].asDouble(), 0.0);
  EXPECT_DOUBLE_EQ(lamp_arm["frames"][1]["T"][2].asDouble(), -5.5);
  // flattened, it is the model as read, with no keys of its own
  shift::core::Context again;
  again.LoadJSON(file);
  const std::string output = Path("models/flat.json");
  shift::core::parameter::ModelWriter::SaveFlattened(
    again.get_model_json(), again.get_filename(), again.get_origins(),
    again.get_files_found(), again.get_files_read(), output);
  shift::core::Context flat;
  flat.LoadJSON(output);
  EXPECT_EQ(flat.get_model_json(), again.get_model_json());
  shift::core::body::Base* a = again.FindActiveBody("a__arm__link1");
  shift::core::body::Base* b = flat.FindActiveBody("a__arm__link1");
  ASSERT_NE(a, nullptr);
  ASSERT_NE(b, nullptr);
  const ksl_SE3_t pa = again.get_body_origin_pose(a);
  const ksl_SE3_t pb = flat.get_body_origin_pose(b);
  for(int k = 0; k < 12; k++) {
    EXPECT_DOUBLE_EQ(pa.as_array[k], pb.as_array[k]) << k;
  }
  EXPECT_DOUBLE_EQ(pa.t.y, -4.0);
}

TEST_F(IncludeTest, ALogKeepsWhereEachIncludedPartCameFrom) {
  const std::string h5 = Path("models/arms.h5");
  shift::core::Context context;
  context.LoadJSON(Model(R"({
    "model_parameters": {
      "gravity": [0.0, 0.0, 9.81], "solvers": [{"type": "rnca"}],
      "integrators": [{"type": "rk4", "time_span": 0.05, "time_step": 0.01}],
      "loggers": [{"type": "hdf5", "logfile": ")" + h5 + R"("}]},
    "assemblies": [{"name": "left", "include": "../left/arm.json"},
                   {"name": "right", "include": "../right/arm.json"}]
  })"));
  context.Simulate();
  // the model logged holds only what the files hold
  {
    std::ifstream in(h5, std::ios::binary);
    const std::string bytes((std::istreambuf_iterator<char>(in)),
                            std::istreambuf_iterator<char>());
    EXPECT_EQ(bytes.find("\"$source\""), std::string::npos);
  }
  shift::gui::Simulation log;
  log.Load(h5);
  ASSERT_TRUE(log.has_results());
  // each arm's link.obj, by the origins the log keeps, taken out of it
  const shift::core::Context* replay = log.get_context();
  EXPECT_EQ(replay->get_origins().get_entries().size(),
            context.get_origins().get_entries().size());
  const auto& read = replay->get_files_read();
  const auto taken_out = std::count_if(
    read.begin(), read.end(), [](const std::string& file) {
      return file.find("link.obj") != std::string::npos &&
             file.rfind(
               (std::filesystem::temp_directory_path() / "shift-log-")
                 .string(),
               0) == 0;
    });
  EXPECT_EQ(taken_out, 2);
  EXPECT_EQ(log.get_scene().meshes.size(), 2u);
}
