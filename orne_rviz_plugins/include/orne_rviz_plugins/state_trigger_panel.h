#ifndef ORNE_RVIZ_PLUGINS__STATE_TRIGGER_PANEL_H
#define ORNE_RVIZ_PLUGINS__STATE_TRIGGER_PANEL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include <QPushButton>
#endif

namespace orne_navigation {

class StateTriggerPanel : public rviz::Panel
{

Q_OBJECT
public:
    StateTriggerPanel(QWidget *parent = 0);
    virtual void load(const rviz::Config &config);
    virtual void save(rviz::Config config) const;

public Q_SLOTS:
    void pushStartWaypointsNav();

protected:
    ros::NodeHandle nh_;
    QPushButton *start_nav_button_;

};

} //namespace orne_navigation

#endif //ORNE_RVIZ_PLUGINS__STATE_TRIGGER_PANEL_H
