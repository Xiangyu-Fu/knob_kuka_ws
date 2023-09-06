#include <QApplication>
#include <QVBoxLayout>
#include <QPushButton>
#include <QWidget>
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/display.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>

int main(int argc, char** argv) {
    QApplication app(argc, argv);

    // Initialize ROS
    ros::init(argc, argv, "rviz_embedded_with_button");

    // Load the URDF into the ROS parameter server
    std::string urdf_file_path = "/home/fu/workspaces/knob_kuka_ws/src/robot_movement_interface/dependencies/ur_description/urdf/ur5_robot.urdf";
    std::ifstream urdf_file(urdf_file_path.c_str());
    std::string urdf_string((std::istreambuf_iterator<char>(urdf_file)), std::istreambuf_iterator<char>());
    ros::param::set("robot_description", urdf_string);

    // Create the main QWidget
    QWidget* main_widget = new QWidget;
    
    QVBoxLayout* layout = new QVBoxLayout;
    main_widget->setLayout(layout);

    // Create and configure the RViz RenderPanel
    rviz::RenderPanel* render_panel = new rviz::RenderPanel;
    layout->addWidget(render_panel);

    rviz::VisualizationManager vis_manager(render_panel);
    vis_manager.setFixedFrame("world");
    render_panel->initialize(vis_manager.getSceneManager(), &vis_manager);

    // Load the URDF model into RViz
    rviz::Display* robot = vis_manager.createDisplay("rviz/RobotModel", "Robot Model", true);
    ROS_ASSERT(robot != nullptr);
    robot->subProp("Robot Description")->setValue("robot_description");

    // Add a QPushButton
    QPushButton* test_button = new QPushButton("Test Button");
    layout->addWidget(test_button);

    // Connect the button's clicked signal to a simple slot (Lambda function in this case)
    QObject::connect(test_button, &QPushButton::clicked, []() {
        ROS_INFO("Test button clicked!");
    });

    main_widget->show();

    return app.exec();
}
