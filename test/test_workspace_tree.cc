/* the model tree of the viewer follows the nesting of the model file, and the
parameters panel lists every parameter of the selected object */
#include "shift/config.h"
#include "shift/gui/workspace.h"

#include <gtest/gtest.h>

#include <json/json.h>

#include <QApplication>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QPushButton>
#include <QTreeWidgetItemIterator>
#include <QTreeWidget>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

/* the window needs an application; the offscreen platform needs no display */
static void EnsureWorkspaceApplication() {
  static int argc = 1;
  static char name[] = "shift-test";
  static char* argv[] = {name, nullptr};
  if(!QApplication::instance()) {
    qputenv("QT_QPA_PLATFORM", "offscreen");
    new QApplication(argc, argv);
  }
}

static std::vector<std::string> ChildTexts(const QTreeWidgetItem* item) {
  std::vector<std::string> texts;
  for(int i = 0; i < item->childCount(); i++) {
    texts.push_back(item->child(i)->text(0).toStdString());
  }
  return texts;
}

static QTreeWidgetItem* Child(QTreeWidgetItem* item, const std::string& text) {
  for(int i = 0; i < item->childCount(); i++) {
    if(item->child(i)->text(0).toStdString() == text) {
      return item->child(i);
    }
  }
  return nullptr;
}

/* the model's loggers write their files into the working directory: run in
one under the build tree */
struct InBuildDirectory {
  std::filesystem::path previous = std::filesystem::current_path();
  InBuildDirectory() {
    const std::filesystem::path directory =
      std::filesystem::path(SHIFT_BINARY_DIR) / "test" / "workspace_tree";
    std::filesystem::create_directories(directory);
    std::filesystem::current_path(directory);
  }
  ~InBuildDirectory() {
    std::filesystem::current_path(previous);
  }
};

/* the model tree, with the columns of joint values and mobility numbers,
and the parameters tree */
static void FindTrees(shift::gui::Workspace& workspace, QTreeWidget*& tree,
                      QTreeWidget*& parameters) {
  tree = parameters = nullptr;
  for(QTreeWidget* widget : workspace.findChildren<QTreeWidget*>()) {
    (widget->columnCount() == 3 ? tree : parameters) = widget;
  }
}

static QTreeWidgetItem* FindItem(QTreeWidget* tree, const QString& text) {
  for(QTreeWidgetItemIterator it(tree); *it; ++it) {
    if((*it)->text(0) == text) {
      return *it;
    }
  }
  return nullptr;
}

/* select the parameter key of the selected object, type text into its
editor and apply it */
static void EditParameter(QTreeWidget* parameters, const QString& key,
                          const QString& text) {
  QTreeWidgetItem* row = FindItem(parameters, key);
  ASSERT_TRUE(row) << key.toStdString();
  parameters->setCurrentItem(row);
  QWidget* panel = parameters->parentWidget();
  auto* line_edit = panel->findChild<QLineEdit*>();
  ASSERT_TRUE(line_edit);
  line_edit->setText(text);
  for(auto* button : panel->findChildren<QPushButton*>()) {
    if(button->text() == "Apply") {
      button->click();
    }
  }
}

TEST(WorkspaceTree, FollowsTheModelFile) {
  EnsureWorkspaceApplication();
  InBuildDirectory directory;
  shift::gui::Workspace workspace;
  QString error;
  ASSERT_TRUE(workspace.Open(
    QString(SHIFT_EXAMPLES_DIR) + "/force_modules/gear_mesh.json", &error))
    << error.toStdString();
  QTreeWidget* tree;
  QTreeWidget* parameters;
  FindTrees(workspace, tree, parameters);
  ASSERT_TRUE(tree);
  ASSERT_TRUE(parameters);

  // the top level of the file: the model parameters, the frames of the fixed
  // body, its collision geometry (the default floor) and the assemblies
  ASSERT_EQ(tree->topLevelItemCount(), 1);
  QTreeWidgetItem* root = tree->topLevelItem(0);
  EXPECT_EQ(root->text(0).toStdString(), "gear_mesh.json");
  EXPECT_EQ(ChildTexts(root),
            (std::vector<std::string>{"model_parameters", "frames",
                                      "collision_geometry", "assemblies"}));
  QTreeWidgetItem* model_parameters = Child(root, "model_parameters");
  EXPECT_EQ(ChildTexts(model_parameters),
            (std::vector<std::string>{"solvers", "integrators", "loggers"}));

  QTreeWidgetItem* assemblies = Child(root, "assemblies");
  ASSERT_TRUE(assemblies);
  ASSERT_EQ(ChildTexts(assemblies), std::vector<std::string>{"gearbox"});
  QTreeWidgetItem* gearbox = assemblies->child(0);
  EXPECT_EQ(ChildTexts(gearbox),
            (std::vector<std::string>{"bodies", "joints", "force_modules",
                                      "sensors", "controllers"}));
  EXPECT_EQ(ChildTexts(Child(gearbox, "bodies")),
            (std::vector<std::string>{"pinion", "wheel"}));
  ASSERT_EQ(Child(gearbox, "sensors")->childCount(), 1);
  ASSERT_EQ(Child(gearbox, "controllers")->childCount(), 1);
  QTreeWidgetItem* mesh = Child(gearbox, "force_modules")->child(0);
  ASSERT_TRUE(mesh);
  EXPECT_EQ(mesh->text(0).toStdString(), "mesh (gear_mesh)");

  // selecting an object lists its parameters, those not in the file too
  tree->setCurrentItem(mesh);
  ASSERT_GT(parameters->topLevelItemCount(), 3);
  std::vector<std::string> keys;
  for(int i = 0; i < parameters->topLevelItemCount(); i++) {
    keys.push_back(parameters->topLevelItem(i)->text(0).toStdString());
  }
  for(const char* key : {"name", "type", "active"}) {
    EXPECT_NE(std::find(keys.begin(), keys.end(), key), keys.end()) << key;
  }
  const auto found = parameters->findItems("name", Qt::MatchExactly, 0);
  ASSERT_EQ(found.size(), 1);
  EXPECT_EQ(found[0]->text(1).toStdString(), "mesh");

  // a folder has no parameters of its own
  tree->setCurrentItem(Child(gearbox, "sensors"));
  EXPECT_EQ(parameters->topLevelItemCount(), 0);
}

TEST(WorkspaceTree, EditReloadsTheModel) {
  EnsureWorkspaceApplication();
  InBuildDirectory directory;
  shift::gui::Workspace workspace;
  QString error;
  ASSERT_TRUE(workspace.Open(
    QString(SHIFT_EXAMPLES_DIR) + "/force_modules/gear_mesh.json", &error))
    << error.toStdString();
  QTreeWidget* tree;
  QTreeWidget* parameters;
  FindTrees(workspace, tree, parameters);
  tree->setCurrentItem(FindItem(tree, "mesh (gear_mesh)"));
  ASSERT_EQ(FindItem(parameters, "pitch_radius_1")->text(1).toStdString(),
            "0.05");

  // read when the model loads: the model is loaded again, with the edit,
  // and the same object and parameter are selected
  EditParameter(parameters, "pitch_radius_1", "0.06");
  ASSERT_TRUE(tree->currentItem());
  EXPECT_EQ(tree->currentItem()->text(0).toStdString(), "mesh (gear_mesh)");
  EXPECT_EQ(FindItem(parameters, "pitch_radius_1")->text(1).toStdString(),
            "0.06");
  EXPECT_EQ(parameters->currentItem()->text(0).toStdString(),
            "pitch_radius_1");
  EXPECT_TRUE(workspace.windowTitle().contains("(edited)"));

  // a value the model rejects is not applied, and the model is as it was
  EditParameter(parameters, "teeth_1", "-3");
  EditParameter(parameters, "contact_ratio", "[1, 2]");
  ASSERT_TRUE(FindItem(parameters, "contact_ratio"));
  EXPECT_EQ(FindItem(parameters, "contact_ratio")->text(1).toStdString(),
            "1.6");
  EXPECT_EQ(FindItem(parameters, "pitch_radius_1")->text(1).toStdString(),
            "0.06");
}

TEST(WorkspaceTree, EditOrientationAsRollPitchYaw) {
  EnsureWorkspaceApplication();
  InBuildDirectory directory;
  shift::gui::Workspace workspace;
  QString error;
  ASSERT_TRUE(workspace.Open(
    QString(SHIFT_EXAMPLES_DIR) + "/force_modules/gear_mesh.json", &error))
    << error.toStdString();
  QTreeWidget* tree;
  QTreeWidget* parameters;
  FindTrees(workspace, tree, parameters);
  tree->setCurrentItem(FindItem(tree, "axis_2"));
  // the keys of a frame's orientation are one row, under the plainest key
  QTreeWidgetItem* row = FindItem(parameters, "R");
  ASSERT_TRUE(row);
  EXPECT_FALSE(FindItem(parameters, "rotation"));
  EXPECT_FALSE(FindItem(parameters, "quaternion"));
  EXPECT_EQ(row->text(1).toStdString(), "roll, pitch, yaw = [0.00, 0.00, 0.00]°");

  parameters->setCurrentItem(row);
  const auto angles =
    parameters->parentWidget()->findChildren<QDoubleSpinBox*>();
  ASSERT_EQ(angles.size(), 3);
  angles[0]->setValue(10.0);
  angles[2]->setValue(90.0);
  for(auto* button : parameters->parentWidget()->findChildren<QPushButton*>()) {
    if(button->text() == "Apply") {
      button->click();
    }
  }
  row = FindItem(parameters, "R");
  ASSERT_TRUE(row);
  EXPECT_EQ(row->text(1).toStdString(),
            "roll, pitch, yaw = [10.00, 0.00, 90.00]°");
}

TEST(WorkspaceTree, RuntimeEditAppliesAtOnce) {
  EnsureWorkspaceApplication();
  InBuildDirectory directory;
  shift::gui::Workspace workspace;
  QString error;
  ASSERT_TRUE(workspace.Open(
    QString(SHIFT_EXAMPLES_DIR) + "/force_modules/powertrain.json", &error))
    << error.toStdString();
  QTreeWidget* tree;
  QTreeWidget* parameters;
  FindTrees(workspace, tree, parameters);
  QTreeWidgetItem* module = nullptr;
  for(QTreeWidgetItemIterator it(tree); *it; ++it) {
    if((*it)->text(0).endsWith("(powertrain)")) {
      module = *it;
    }
  }
  ASSERT_TRUE(module);
  tree->setCurrentItem(module);
  EditParameter(parameters, "throttle", "0.5");
  // set on the object as it is: the model is not loaded again
  EXPECT_EQ(tree->currentItem(), module);
  EXPECT_EQ(FindItem(parameters, "throttle")->text(1).toStdString(), "0.5");
  EXPECT_TRUE(workspace.windowTitle().contains("(edited)"));
}

TEST(WorkspaceTree, EditKeepsTheFormOfTheOrientation) {
  EnsureWorkspaceApplication();
  InBuildDirectory directory;
  // gear_mesh.json, with the orientation of a frame given as a quaternion
  Json::Value model;
  std::ifstream(std::string(SHIFT_EXAMPLES_DIR) +
                "/force_modules/gear_mesh.json") >>
    model;
  for(auto& frame : model["frames"]) {
    if(frame["name"].asString() == "axis_2") {
      frame["quaternion"] = Json::Value(Json::arrayValue);
      for(double value : {0.0, 0.0, 0.0, 1.0}) {
        frame["quaternion"].append(value);
      }
    }
  }
  const std::string path =
    (std::filesystem::current_path() / "gear_mesh_quaternion.json").string();
  std::ofstream(path) << model;

  shift::gui::Workspace workspace;
  QString error;
  ASSERT_TRUE(workspace.Open(QString::fromStdString(path), &error))
    << error.toStdString();
  QTreeWidget* tree;
  QTreeWidget* parameters;
  FindTrees(workspace, tree, parameters);
  tree->setCurrentItem(FindItem(tree, "axis_2"));
  // the row of the orientation has the key the file gives
  QTreeWidgetItem* row = FindItem(parameters, "quaternion");
  ASSERT_TRUE(row);
  EXPECT_FALSE(FindItem(parameters, "R"));
  parameters->setCurrentItem(row);
  const auto angles =
    parameters->parentWidget()->findChildren<QDoubleSpinBox*>();
  ASSERT_EQ(angles.size(), 3);
  angles[1]->setValue(30.0);
  for(auto* button : parameters->parentWidget()->findChildren<QPushButton*>()) {
    if(button->text() == "Apply") {
      button->click();
    }
  }
  // loaded again, still given as a quaternion
  row = FindItem(parameters, "quaternion");
  ASSERT_TRUE(row);
  EXPECT_EQ(row->text(1).toStdString(),
            "roll, pitch, yaw = [0.00, 30.00, 0.00]°");
}

static Json::Value ReadJson(const std::string& path) {
  Json::Value json;
  std::ifstream(path) >> json;
  return json;
}

/* the names of the elements of every array, by the path of the array */
static void ArrayOrders(const Json::Value& json, const std::string& path,
                        std::vector<std::string>& orders) {
  if(json.isArray()) {
    std::string names;
    for(Json::ArrayIndex i = 0; i < json.size(); i++) {
      if(json[i].isObject() && json[i].isMember("name")) {
        names += json[i]["name"].asString() + " ";
      }
      ArrayOrders(json[i], path + "/" + std::to_string(i), orders);
    }
    orders.push_back(path + ": " + names);
  } else if(json.isObject()) {
    for(const auto& key : json.getMemberNames()) {
      ArrayOrders(json[key], path + "/" + key, orders);
    }
  }
}

TEST(WorkspaceTree, SavedModelKeepsTheOrderOfArrays) {
  EnsureWorkspaceApplication();
  InBuildDirectory directory;
  // a model with many arrays: saved without edits it is the model file
  {
    shift::gui::Workspace workspace;
    QString error;
    const std::string original =
      std::string(SHIFT_EXAMPLES_DIR) + "/raptor/raptor.json";
    ASSERT_TRUE(workspace.Open(QString::fromStdString(original), &error))
      << error.toStdString();
    ASSERT_TRUE(workspace.SaveModel("raptor_saved.json", &error))
      << error.toStdString();
    EXPECT_EQ(ReadJson("raptor_saved.json"), ReadJson(original));
  }
  // with an edit: only the edited value differs, every array in its order
  shift::gui::Workspace workspace;
  QString error;
  const std::string original =
    std::string(SHIFT_EXAMPLES_DIR) + "/force_modules/gear_mesh.json";
  ASSERT_TRUE(workspace.Open(QString::fromStdString(original), &error))
    << error.toStdString();
  QTreeWidget* tree;
  QTreeWidget* parameters;
  FindTrees(workspace, tree, parameters);
  tree->setCurrentItem(FindItem(tree, "wheel_bearing (revolute)"));
  ASSERT_TRUE(tree->currentItem());
  QTreeWidgetItem* row = FindItem(parameters, "initial_displacement");
  ASSERT_TRUE(row);
  EditParameter(parameters, "initial_displacement", "0.25");
  ASSERT_TRUE(workspace.SaveModel("gear_mesh_saved.json", &error))
    << error.toStdString();
  Json::Value expected = ReadJson(original);
  for(auto& joint : expected["assemblies"][0]["joints"]) {
    if(joint["name"].asString() == "wheel_bearing") {
      joint["initial_displacement"] = 0.25;
    }
  }
  const Json::Value saved = ReadJson("gear_mesh_saved.json");
  std::vector<std::string> saved_orders, expected_orders;
  ArrayOrders(saved, "", saved_orders);
  ArrayOrders(expected, "", expected_orders);
  EXPECT_EQ(saved_orders, expected_orders);
  EXPECT_EQ(saved, expected);
}
