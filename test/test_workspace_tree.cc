/* the model tree of the viewer follows the nesting of the model file, and the
parameters panel lists every parameter of the selected object */
#include "shift/config.h"
#include "shift/gui/settings.h"
#include "shift/gui/viewport.h"
#include "shift/gui/workspace.h"

#include <gtest/gtest.h>

#include <json/json.h>

#include <QApplication>
#include <QColor>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMouseEvent>
#include <QPushButton>
#include <QTreeWidgetItemIterator>
#include <QTreeWidget>

#include <algorithm>
#include <cmath>
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

#if SHIFT_HAVE_FEA
/* the body panel of a flexible body built from a mesh: its mesh sizes and
the modes checked in the mode window are applied together by its Apply,
which loads the model again */
TEST(WorkspaceTree, FlexApplyWritesMeshSizesAndModes) {
  EnsureWorkspaceApplication();
  InBuildDirectory directory;
  const std::string base = testing::TempDir() + "/flex_apply";
  const std::string path = base + ".json";
  {
    std::ofstream file(path);
    file << R"({"model_parameters": {"version": "0.2",
      "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
                "time": "seconds"},
      "gravity": [0, 9.81, 0], "solvers": [{"type": "rnca"}],
      "include_directories": [")"
         << SHIFT_EXAMPLES_DIR << R"(/cad"],
      "integrators": [{"type": "ode113", "time_span": 0.002,
                       "time_step": 0.001}]},
      "assemblies": [{"name": "test", "bodies": [{"name": "plate",
        "type": "modal", "mode_count": 2, "fe_cache": ")"
         << base << R"(.fe.h5",
        "density": 7850, "youngs_modulus": 2.1e11, "poisson_ratio": 0.3,
        "fe_geometry": "solid", "element_size": 0.02,
        "frames": [{"name": "pin", "T": [0, 0, 0.01]}],
        "geometry": [{"name": "solid", "type": "cad",
                      "filename": "plate_hole.step"}]}],
      "joints": [{"name": "hinge", "type": "revolute",
                  "parent": [["fixed"], "origin"],
                  "child": [["plate"], "pin"], "axis": [0, 0, 1]}]}]})";
  }
  std::remove((base + ".fe.h5").c_str());
  shift::gui::Workspace workspace;
  QString error;
  ASSERT_TRUE(workspace.Open(QString::fromStdString(path), &error))
    << error.toStdString();
  QTreeWidget* tree;
  QTreeWidget* parameters;
  FindTrees(workspace, tree, parameters);
  auto select_plate = [&]() {
    for(QTreeWidgetItemIterator it(tree); *it; ++it) {
      if((*it)->text(0).startsWith("plate")) {
        tree->setCurrentItem(*it);
        return;
      }
    }
    FAIL() << "no plate";
  };
  select_plate();
  QLineEdit* size = nullptr;
  QLineEdit* minimum = nullptr;
  QPushButton* apply = nullptr;
  QPushButton* modes = nullptr;
  auto find_widgets = [&]() {
    for(auto* edit : workspace.findChildren<QLineEdit*>()) {
      if(edit->toolTip().startsWith("element_size")) {
        size = edit;
      } else if(edit->toolTip().startsWith("minimum_element_size")) {
        minimum = edit;
      }
    }
    for(auto* button : workspace.findChildren<QPushButton*>()) {
      if(button->toolTip().startsWith("write the changes to this flexible")) {
        apply = button;
      } else if(button->text() == "Modes...") {
        modes = button;
      }
    }
  };
  find_widgets();
  ASSERT_TRUE(size && minimum && apply && modes);
  EXPECT_EQ(size->text().toStdString(), "0.02");
  EXPECT_TRUE(minimum->text().isEmpty());
  /* the legend of the attachments: the pin's holds the plate */
  bool legend = false;
  for(auto* label : workspace.findChildren<QLabel*>()) {
    if(label->text().startsWith("<b>Attachments</b>")) {
      legend = true;
      EXPECT_TRUE(label->text().contains(
        "<b>pin</b> (revolute joint &#x27;hinge&#x27;): averaging (RBE3)") ||
        label->text().contains(
        "<b>pin</b> (revolute joint 'hinge'): averaging (RBE3)"))
        << label->text().toStdString();
      EXPECT_TRUE(label->text().contains("the reference frame, held"));
    }
  }
  EXPECT_TRUE(legend);
  EXPECT_FALSE(apply->isEnabled());

  // a minimum above the size, or no size, cannot be applied
  size->setText("0.015");
  EXPECT_TRUE(apply->isEnabled());
  minimum->setText("0.03");
  EXPECT_FALSE(apply->isEnabled());
  minimum->setText("0.003");
  EXPECT_TRUE(apply->isEnabled());

  // one mode unchecked in the mode window, then one Apply for both
  modes->click();
  QListWidget* list = nullptr;
  for(auto* widget : workspace.findChildren<QListWidget*>()) {
    if(widget->window() != &workspace) {
      list = widget;
    }
  }
  ASSERT_TRUE(list);
  QListWidgetItem* mode_1 = nullptr;
  for(int row = 0; row < list->count(); row++) {
    if(list->item(row)->text().startsWith("mode 1:")) {
      mode_1 = list->item(row);
    }
  }
  ASSERT_TRUE(mode_1);
  EXPECT_EQ(mode_1->checkState(), Qt::Checked);
  mode_1->setCheckState(Qt::Unchecked);
  apply->click();

  EXPECT_TRUE(workspace.windowTitle().contains("(edited)"));
  ASSERT_TRUE(tree->currentItem());
  EXPECT_TRUE(tree->currentItem()->text(0).startsWith("plate"));
  ASSERT_TRUE(FindItem(parameters, "element_size"));
  EXPECT_EQ(FindItem(parameters, "element_size")->text(1).toStdString(),
            "0.015");
  ASSERT_TRUE(FindItem(parameters, "minimum_element_size"));
  EXPECT_EQ(
    FindItem(parameters, "minimum_element_size")->text(1).toStdString(),
    "0.003");
  ASSERT_TRUE(FindItem(parameters, "modes_kept"));
  EXPECT_EQ(FindItem(parameters, "modes_kept")->text(1).toStdString(), "[0]");

  // the panel shows the model's values again, with nothing to apply
  find_widgets();
  EXPECT_EQ(size->text().toStdString(), "0.015");
  EXPECT_EQ(minimum->text().toStdString(), "0.003");
  EXPECT_FALSE(apply->isEnabled());

  // the minimum cleared goes back to its default
  minimum->setText("");
  EXPECT_TRUE(apply->isEnabled());
  apply->click();
  find_widgets();
  EXPECT_TRUE(minimum->text().isEmpty());
  EXPECT_FALSE(apply->isEnabled());

  std::remove(path.c_str());
  std::remove((base + ".fe.h5").c_str());
}
#endif

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

/* the frames on the bodies the viewport draws */
static size_t FramesShown(const shift::gui::Workspace& workspace) {
  return workspace.get_viewport()->get_frame_axes().size();
}

/* whether the axes of an item are shown: 0 no, 1 yes, 2 some of a folder's;
-1 if it has no eye */
static int Eye(const QTreeWidgetItem* item) {
  const QVariant visible = item->data(0, Qt::UserRole + 4);
  return visible.isValid() ? visible.toInt() : -1;
}

/* click the eye of an item, as the user does: along the row from its left
until a click changes it; the icon before the eye takes no clicks */
static void ClickEye(QTreeWidget* tree, QTreeWidgetItem* item) {
  tree->scrollToItem(item);
  const QRect rect = tree->visualItemRect(item);
  const int before = Eye(item);
  for(int x = rect.left(); x < rect.left() + 120; x += 2) {
    const QPointF point(x, rect.center().y());
    for(QEvent::Type type :
        {QEvent::MouseButtonPress, QEvent::MouseButtonRelease}) {
      QMouseEvent event(type, point, tree->viewport()->mapToGlobal(point),
                        Qt::LeftButton,
                        type == QEvent::MouseButtonPress ? Qt::LeftButton
                                                         : Qt::NoButton,
                        Qt::NoModifier);
      QApplication::sendEvent(tree->viewport(), &event);
    }
    if(Eye(item) != before) {
      return;
    }
  }
  ADD_FAILURE() << "no eye on " << item->text(0).toStdString();
}

TEST(WorkspaceTree, EyeShowsTheAxesOfFrames) {
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

  // the frames of the fixed body and of each body, all hidden at first;
  // bodies have an eye, shown; the model file has none
  QTreeWidgetItem* root = tree->topLevelItem(0);
  QTreeWidgetItem* fixed_frames = Child(root, "frames");
  ASSERT_TRUE(fixed_frames);
  EXPECT_EQ(Eye(fixed_frames), 0);
  QTreeWidgetItem* axis_2 = Child(fixed_frames, "axis_2");
  ASSERT_TRUE(axis_2);
  EXPECT_EQ(Eye(axis_2), 0);
  QTreeWidgetItem* pinion = FindItem(tree, "pinion");
  ASSERT_TRUE(pinion);
  QTreeWidgetItem* pinion_frames = Child(pinion, "frames");
  ASSERT_TRUE(pinion_frames);
  EXPECT_EQ(Eye(pinion_frames), 0);
  EXPECT_EQ(Eye(Child(pinion_frames, "origin")), 0);
  EXPECT_EQ(Eye(pinion), 1);
  EXPECT_EQ(Eye(root), -1);
  EXPECT_EQ(FramesShown(workspace), 0u);

  // one frame
  ClickEye(tree, axis_2);
  EXPECT_EQ(Eye(axis_2), 1);
  ASSERT_EQ(FramesShown(workspace), 1u);
  const auto& shown = workspace.get_viewport()->get_frame_axes().front();
  EXPECT_EQ(shown.body_index, 0);
  EXPECT_FLOAT_EQ(shown.frame.t.x, 0.15f);
  EXPECT_EQ(Eye(fixed_frames), Child(fixed_frames, "origin") ? 2 : 1);

  // a folder shows all of its frames, then hides them
  ClickEye(tree, pinion_frames);
  EXPECT_EQ(Eye(pinion_frames), 1);
  EXPECT_EQ(Eye(Child(pinion_frames, "origin")), 1);
  const size_t with_pinion = FramesShown(workspace);
  EXPECT_EQ(with_pinion, 1u + pinion_frames->childCount());

  // the frames shown stay shown when an edit loads the model again
  tree->setCurrentItem(FindItem(tree, "mesh (gear_mesh)"));
  EditParameter(parameters, "pitch_radius_1", "0.06");
  EXPECT_EQ(FramesShown(workspace), with_pinion);
  pinion_frames = Child(FindItem(tree, "pinion"), "frames");
  EXPECT_EQ(Eye(pinion_frames), 1);

  ClickEye(tree, pinion_frames);
  EXPECT_EQ(Eye(pinion_frames), 0);
  EXPECT_EQ(FramesShown(workspace), 1u);

  // opening a model shows none
  ASSERT_TRUE(workspace.Open(
    QString(SHIFT_EXAMPLES_DIR) + "/force_modules/gear_mesh.json", &error));
  EXPECT_EQ(FramesShown(workspace), 0u);
}

TEST(WorkspaceTree, EyeShowsJointAxesAndHidesBodies) {
  EnsureWorkspaceApplication();
  InBuildDirectory directory;
  shift::gui::Workspace workspace;
  QString error;
  ASSERT_TRUE(workspace.Open(
    QString(SHIFT_EXAMPLES_DIR) + "/joints/screw_and_gear.json", &error))
    << error.toStdString();
  QTreeWidget* tree;
  QTreeWidget* parameters;
  FindTrees(workspace, tree, parameters);
  const shift::gui::Viewport* viewport = workspace.get_viewport();

  // joints with an axis have an eye, hidden at first; a rigid joint has none
  QTreeWidgetItem* bearing = FindItem(tree, "gear_a_bearing (revolute)");
  QTreeWidgetItem* gear = FindItem(tree, "gear_a_to_gear_b (gear)");
  QTreeWidgetItem* screw = FindItem(tree, "screw_right (screw)");
  QTreeWidgetItem* rigid = FindItem(tree, "board_fixed (rigid)");
  ASSERT_TRUE(bearing && gear && screw && rigid);
  EXPECT_EQ(Eye(bearing), 0);
  EXPECT_EQ(Eye(gear), 0);
  EXPECT_EQ(Eye(screw), 0);
  EXPECT_EQ(Eye(rigid), -1);
  EXPECT_TRUE(viewport->get_joint_axes().empty());

  // a revolute joint: one line, along the z axis of its parent frame on the
  // fixed body
  ClickEye(tree, bearing);
  ASSERT_EQ(viewport->get_joint_axes().size(), 1u);
  EXPECT_EQ(viewport->get_joint_axes()[0].body_index, 0);
  EXPECT_EQ(Eye(bearing->parent()), 2);

  // a gear joint: the axes of both gears, on their bodies
  ClickEye(tree, gear);
  ASSERT_EQ(viewport->get_joint_axes().size(), 3u);
  const int gear_a = viewport->get_joint_axes()[1].body_index;
  const int gear_b = viewport->get_joint_axes()[2].body_index;
  EXPECT_NE(gear_a, 0);
  EXPECT_NE(gear_b, 0);
  EXPECT_NE(gear_a, gear_b);
  // in the world the axis of gear a is that of its bearing: the same line
  // (the poses come with the first frame drawn, which offscreen never is)
  const_cast<shift::gui::Viewport*>(viewport)->UpdatePoses();
  shift::gui::Poses poses;
  viewport->get_poses(poses);
  ASSERT_FALSE(poses.bodies.empty());
  auto world = [&](const shift::gui::Viewport::JointAxis& axis) {
    ksl_SE3f_t pose;
    ksl_product_ddf(&poses.bodies[axis.body_index], &axis.line, &pose);
    return std::make_pair(pose.t, pose.R.v2);
  };
  const auto [bearing_point, bearing_axis] =
    world(viewport->get_joint_axes()[0]);
  const auto [gear_point, gear_axis] = world(viewport->get_joint_axes()[1]);
  EXPECT_NEAR(std::fabs(ksl_dot_vvf(&bearing_axis, &gear_axis)), 1.f, 1e-5f);
  ksl_vec3f_t apart;
  ksl_subtract_vvf(&bearing_point, &gear_point, &apart);
  EXPECT_NEAR(ksl_vec3f_l2norm(&apart), 0.f, 1e-5f);

  // the joint color, purple
  EXPECT_NEAR(viewport->get_joint_color().x, 0xa0 / 255.f, 1e-6f);

  // hiding a body; then all of them with their folder, and back
  QTreeWidgetItem* body = FindItem(tree, "gear_a");
  ASSERT_TRUE(body);
  EXPECT_EQ(Eye(body), 1);
  ClickEye(tree, body);
  EXPECT_EQ(Eye(body), 0);
  const std::vector<bool>& hidden = viewport->get_hidden_bodies();
  ASSERT_EQ(std::count(hidden.begin(), hidden.end(), true), 1);
  EXPECT_TRUE(hidden[gear_a]);
  EXPECT_EQ(Eye(body->parent()), 2);
  ClickEye(tree, body->parent());
  EXPECT_EQ(Eye(body->parent()), 1);
  EXPECT_EQ(std::count(viewport->get_hidden_bodies().begin(),
                       viewport->get_hidden_bodies().end(), true),
            0);
  ClickEye(tree, body->parent());
  EXPECT_EQ(std::count(viewport->get_hidden_bodies().begin(),
                       viewport->get_hidden_bodies().end(), true),
            body->parent()->childCount());
}

/* the world direction of the arrow of joint axis i, +z of its line */
static ksl_vec3f_t ArrowDirection(shift::gui::Workspace& workspace,
                                  size_t i) {
  auto* viewport = const_cast<shift::gui::Viewport*>(workspace.get_viewport());
  viewport->UpdatePoses();
  shift::gui::Poses poses;
  viewport->get_poses(poses);
  const auto& axis = viewport->get_joint_axes().at(i);
  ksl_SE3f_t pose;
  ksl_product_ddf(&poses.bodies.at(axis.body_index), &axis.line, &pose);
  ksl_vec3f_t z = pose.R.v2;
  ksl_vec3f_normalize(&z);
  return z;
}

TEST(WorkspaceTree, JointArrowPointsAlongTheGivenAxis) {
  EnsureWorkspaceApplication();
  InBuildDirectory directory;
  shift::gui::Workspace workspace;
  QString error;
  ASSERT_TRUE(workspace.Open(
    QString(SHIFT_EXAMPLES_DIR) + "/joints/screw_and_gear.json", &error))
    << error.toStdString();
  QTreeWidget* tree;
  QTreeWidget* parameters;
  FindTrees(workspace, tree, parameters);
  QTreeWidgetItem* bearing = FindItem(tree, "gear_a_bearing (revolute)");
  ASSERT_TRUE(bearing);
  ClickEye(tree, bearing);
  ASSERT_EQ(workspace.get_viewport()->get_joint_axes().size(), 1u);
  const ksl_vec3f_t given = ArrowDirection(workspace, 0);

  // the axis the file gives, reversed: the arrow turns around
  tree->setCurrentItem(bearing);
  EditParameter(parameters, "axis", "[0, 0, -1]");
  ASSERT_EQ(workspace.get_viewport()->get_joint_axes().size(), 1u);
  const ksl_vec3f_t reversed = ArrowDirection(workspace, 0);
  EXPECT_NEAR(ksl_dot_vvf(&reversed, &given), -1.f, 1e-5f);
}

TEST(WorkspaceTree, SettingsPersist) {
  EnsureWorkspaceApplication();
  InBuildDirectory directory;
  const std::string path =
    (std::filesystem::current_path() / "settings" / "settings.json").string();
  std::filesystem::remove_all(std::filesystem::current_path() / "settings");

  // no file yet: the defaults
  shift::gui::Settings settings;
  ASSERT_TRUE(settings.Load(path));
  EXPECT_EQ(settings.get_frame_scale(),
            shift::gui::Settings::kDefaultFrameScale);

  // saved, with its directory made, keeping keys it does not know
  settings.set_frame_scale(0.5);
  ASSERT_TRUE(settings.Save(path));
  {
    std::ifstream file(path);
    Json::Value json;
    file >> json;
    json["from_a_later_version"] = 7;
    std::ofstream(path) << json;
  }
  shift::gui::Settings read;
  ASSERT_TRUE(read.Load(path));
  EXPECT_EQ(read.get_frame_scale(), 0.5);
  read.set_frame_scale(0.75);
  ASSERT_TRUE(read.Save(path));
  std::ifstream file(path);
  Json::Value json;
  file >> json;
  // grouped as the pages of the Settings dialog
  EXPECT_EQ(json["interface"]["frame_scale"].asDouble(), 0.75);
  EXPECT_EQ(json["interface"]["joint_color"].asString(), "#a040e0");
  EXPECT_EQ(json["interface"]["joint_thickness"].asDouble(),
            shift::gui::Settings::kDefaultJointThickness);
  EXPECT_EQ(json["from_a_later_version"].asInt(), 7);
  EXPECT_FALSE(json.isMember("frame_scale"));

  EXPECT_EQ(json["rendering"]["lod_tolerance"].asDouble(),
            shift::gui::Settings::kDefaultLodTolerance);
  EXPECT_EQ(json["rendering"]["tone_mapping"].asString(), "reinhard");
  EXPECT_EQ(json["flexible_bodies"]["stress_quantity"].asString(),
            "von_mises");

  // the level of detail tolerance, 0 (the meshes themselves) allowed
  read.set_lod_tolerance(0.0);
  ASSERT_TRUE(read.Save(path));
  shift::gui::Settings full;
  ASSERT_TRUE(full.Load(path));
  EXPECT_EQ(full.get_lod_tolerance(), 0.0);
  read.set_lod_tolerance(shift::gui::Settings::kDefaultLodTolerance);

  // the ray tracer's render scale, from kMinimumRenderScale to 1
  EXPECT_EQ(json["rendering"]["render_scale"].asDouble(),
            shift::gui::Settings::kDefaultRenderScale);
  read.set_render_scale(0.5);
  ASSERT_TRUE(read.Save(path));
  shift::gui::Settings scaled;
  ASSERT_TRUE(scaled.Load(path));
  EXPECT_EQ(scaled.get_render_scale(), 0.5);
  read.set_render_scale(shift::gui::Settings::kDefaultRenderScale);

  // the joint color, as #rrggbb
  read.set_joint_color(QColor(0x20, 0x80, 0x40));
  ASSERT_TRUE(read.Save(path));
  shift::gui::Settings colored;
  ASSERT_TRUE(colored.Load(path));
  EXPECT_EQ(colored.get_joint_color(), QColor(0x20, 0x80, 0x40));
  std::ofstream(path) << "{\"joint_color\": \"not a color\"}";
  shift::gui::Settings bad_color;
  EXPECT_FALSE(bad_color.Load(path));
  EXPECT_EQ(bad_color.get_joint_color(),
            shift::gui::Settings::get_default_joint_color());

  // the thickness of the joint axes
  read.set_joint_thickness(0.02);
  ASSERT_TRUE(read.Save(path));
  shift::gui::Settings thick;
  ASSERT_TRUE(thick.Load(path));
  EXPECT_EQ(thick.get_joint_thickness(), 0.02);
  std::ofstream(path) << "{\"joint_thickness\": 0}";
  shift::gui::Settings bad_thickness;
  std::string thickness_error;
  EXPECT_FALSE(bad_thickness.Load(path, &thickness_error));
  EXPECT_NE(thickness_error.find("joint_thickness"), std::string::npos);

  std::ofstream(path) << "{\"lod_tolerance\": -1}";
  shift::gui::Settings bad_tolerance;
  std::string tolerance_error;
  EXPECT_FALSE(bad_tolerance.Load(path, &tolerance_error));
  EXPECT_NE(tolerance_error.find("lod_tolerance"), std::string::npos);

  for(const char* scale : {"0.1", "1.5", "\"half\""}) {
    std::ofstream(path) << "{\"render_scale\": " << scale << "}";
    shift::gui::Settings bad_scale;
    std::string scale_error;
    EXPECT_FALSE(bad_scale.Load(path, &scale_error)) << scale;
    EXPECT_NE(scale_error.find("render_scale"), std::string::npos);
    EXPECT_EQ(bad_scale.get_render_scale(),
              shift::gui::Settings::kDefaultRenderScale);
  }

  // DLSS for the ray traced view: on, in Quality mode, by default
  EXPECT_TRUE(thick.is_dlss_enabled());
  EXPECT_EQ(thick.get_dlss_quality(), shift::gui::Dlss::Quality::kQuality);
  thick.set_dlss_enabled(false);
  thick.set_dlss_quality(shift::gui::Dlss::Quality::kUltraPerformance);
  ASSERT_TRUE(thick.Save(path));
  {
    std::ifstream saved_file(path);
    Json::Value saved_json;
    saved_file >> saved_json;
    EXPECT_FALSE(saved_json["rendering"]["dlss"].asBool());
    EXPECT_EQ(saved_json["rendering"]["dlss_mode"].asString(),
              "ultra_performance");
  }
  shift::gui::Settings upscaled;
  ASSERT_TRUE(upscaled.Load(path));
  EXPECT_FALSE(upscaled.is_dlss_enabled());
  EXPECT_EQ(upscaled.get_dlss_quality(),
            shift::gui::Dlss::Quality::kUltraPerformance);
  std::ofstream(path) << "{\"dlss_mode\": \"fast\"}";
  shift::gui::Settings bad_mode;
  std::string mode_error;
  EXPECT_FALSE(bad_mode.Load(path, &mode_error));
  EXPECT_NE(mode_error.find("dlss_mode"), std::string::npos);

  // the tone curve, the exposure, how terrain and soil are drawn, and how
  // flexible bodies are drawn
  thick.set_tone_aces(true);
  thick.set_exposure(-1.5);
  thick.set_smooth_terrain(false);
  thick.set_soil_surface(false);
  thick.set_deformation_scale(20.0);
  thick.set_flex_mesh_surface(true);
  thick.set_flex_element_edges(true);
  thick.set_flex_constrained_nodes(true);
  thick.set_flex_stresses(true);
  ASSERT_TRUE(thick.Save(path));
  shift::gui::Settings drawn;
  ASSERT_TRUE(drawn.Load(path));
  EXPECT_TRUE(drawn.is_tone_aces());
  EXPECT_EQ(drawn.get_exposure(), -1.5);
  EXPECT_FALSE(drawn.is_smooth_terrain());
  EXPECT_FALSE(drawn.is_soil_surface());
  EXPECT_EQ(drawn.get_deformation_scale(), 20.0);
  EXPECT_TRUE(drawn.is_flex_mesh_surface());
  EXPECT_TRUE(drawn.is_flex_element_edges());
  EXPECT_TRUE(drawn.is_flex_constrained_nodes());
  EXPECT_TRUE(drawn.is_flex_stresses());
  std::ofstream(path) << "{\"rendering\": {\"tone_mapping\": \"filmic\"}}";
  shift::gui::Settings bad_tone;
  std::string tone_error;
  EXPECT_FALSE(bad_tone.Load(path, &tone_error));
  EXPECT_NE(tone_error.find("tone_mapping"), std::string::npos);

  // a file of an earlier version, its keys at the top level, is read and
  // written grouped
  std::ofstream(path) << "{\"frame_scale\": 0.3, \"render_scale\": 0.5}";
  shift::gui::Settings earlier;
  ASSERT_TRUE(earlier.Load(path));
  EXPECT_EQ(earlier.get_frame_scale(), 0.3);
  EXPECT_EQ(earlier.get_render_scale(), 0.5);
  ASSERT_TRUE(earlier.Save(path));
  {
    std::ifstream grouped_file(path);
    Json::Value grouped;
    grouped_file >> grouped;
    EXPECT_FALSE(grouped.isMember("frame_scale"));
    EXPECT_EQ(grouped["interface"]["frame_scale"].asDouble(), 0.3);
    EXPECT_EQ(grouped["rendering"]["render_scale"].asDouble(), 0.5);
  }

  // a bad value is reported and the defaults kept
  std::ofstream(path) << "{\"frame_scale\": -1}";
  shift::gui::Settings bad;
  std::string error;
  EXPECT_FALSE(bad.Load(path, &error));
  EXPECT_NE(error.find("frame_scale"), std::string::npos);
  EXPECT_EQ(bad.get_frame_scale(), shift::gui::Settings::kDefaultFrameScale);

  // the viewer reads ~/.shift/settings.json at start
  const QByteArray home = qgetenv("HOME");
  const std::filesystem::path fake_home =
    std::filesystem::current_path() / "home";
  std::filesystem::remove_all(fake_home);
  qputenv("HOME", QByteArray::fromStdString(fake_home.string()));
  shift::gui::Settings saved;
  saved.set_frame_scale(1.5);
  saved.set_joint_color(QColor(255, 0, 0));
  saved.set_joint_thickness(0.03);
  saved.set_dlss_enabled(false);
  saved.set_dlss_quality(shift::gui::Dlss::Quality::kBalanced);
  ASSERT_TRUE(saved.Save(shift::gui::Settings::get_default_path()));
  EXPECT_TRUE(std::filesystem::exists(fake_home / ".shift" / "settings.json"));
  {
    shift::gui::Workspace workspace;
    EXPECT_EQ(workspace.get_settings().get_frame_scale(), 1.5);
    EXPECT_EQ(workspace.get_viewport()->get_frame_axes_scale(), 1.5f);
    const ksl_vec3f_t color = workspace.get_viewport()->get_joint_color();
    EXPECT_EQ(color.x, 1.f);
    EXPECT_EQ(color.y, 0.f);
    EXPECT_EQ(color.z, 0.f);
    EXPECT_EQ(workspace.get_viewport()->get_joint_thickness(), 0.03f);
    EXPECT_FALSE(workspace.get_viewport()->is_dlss_enabled());
    EXPECT_EQ(workspace.get_viewport()->get_dlss_quality(),
              shift::gui::Dlss::Quality::kBalanced);
  }
  qputenv("HOME", home);
}
