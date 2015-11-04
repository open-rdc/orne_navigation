#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QtGui/QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <geometry_msgs/Twist.h>

#include "teleop_panel.h"

namespace rviz_plugin_tutorials
{

TeleopPanel::TeleopPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  start_nav_button_ = new QPushButton("StartWaypointsNavigation");

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(start_nav_button_);
  setLayout( layout );
  
  connect(start_nav_button_, SIGNAL(clicked()), this, SLOT(pushStartNavigation()));
}

void TeleopPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

void TeleopPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}

void TeleopPanel::pushStartNavigation() {
    ROS_ERROR("just debugging");
}

} // end namespace rviz_plugin_tutorials

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::TeleopPanel,rviz::Panel )
