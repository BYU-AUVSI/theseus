#ifndef GROUNDSTATION_PANEL_H
#define GROUNDSTATION_PANEL_H

#ifndef Q_MOC_RUN
  #include <ros/ros.h>
  #include <rviz/panel.h>
#endif
#include <QGroupBox>
#include <QRadioButton>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QGroupBox>

#include <std_srvs/Trigger.h>

class QLineEdit;
class QCheckBox;
class QString;

enum flight_mode_state
{
  FLY,
  RETURN_TO_HOME,
  TERMINATE_FLIGHT
};

namespace theseus
{
  class GroundstationPanel : public rviz::Panel
  {
  Q_OBJECT
  public:
    GroundstationPanel(QWidget* parent);

    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

  public Q_SLOTS:
    void setFlyEnabled(bool enabled);
    void setRTHEnabled(bool enabled);
    void setTerminateEnabled(bool enabled);

  protected:
    QGroupBox *createExclusiveGroup();
    QRadioButton *radio1, *radio2, *radio3;
    ros::NodeHandle nh_;

    ros::ServiceClient terminate_client_;
    ros::ServiceClient save_flight_client_;
    ros::ServiceClient return_to_home_client_;
    ros::ServiceClient resume_path_client_;
    bool fly_enabled_;
    bool rth_enabled_;
    bool terminate_enabled_;
    flight_mode_state flight_mode_;

    void failsafeCallback(flight_mode_state state);
};

} // namespace theseus

#endif // GROUNDSTATION_PANEL_H
