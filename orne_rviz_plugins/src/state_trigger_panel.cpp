
#include <orne_rviz_plugins/state_trigger_panel.h>

#include <QHBoxLayout>

namespace orne_rviz_plugins
{

StateTriggerPanel::StateTriggerPanel(QWidget *parent) : 
    rviz::Panel(parent)
{
    QHBoxLayout *layout = new QHBoxLayout;
    start_nav_button_ = new QPushButton("StartWaypointsNav");
    layout->addWidget(start_nav_button_);
    setLayout(layout);

    connect(start_nav_button_, SIGNAL(clicked()), this, SLOT(pushStartWaypointsNav()));
}

void StateTriggerPanel::pushStartWaypointsNav() {
    ROS_ERROR("DEBUG: panel!!!");
}

void StateTriggerPanel::load(const rviz::Config &config) {
    rviz::Panel::load(config);

}

void StateTriggerPanel::save(rviz::Config config) const {
    rviz::Panel::save(config);
}

} //namespace orne_navigation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(orne_rviz_plugins::StateTriggerPanel, rviz::Panel)

