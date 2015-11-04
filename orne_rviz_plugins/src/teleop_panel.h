#ifndef TELEOP_PANEL_H
#define TELEOP_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

class QPushButton;

namespace rviz_plugin_tutorials
{

class TeleopPanel: public rviz::Panel
{
Q_OBJECT
public:
  TeleopPanel( QWidget* parent = 0 );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

public Q_SLOTS:
  void pushStartNavigation();
    
protected:
  ros::NodeHandle nh_;
  QPushButton *start_nav_button_;

};

} // end namespace rviz_plugin_tutorials

#endif // TELEOP_PANEL_H
