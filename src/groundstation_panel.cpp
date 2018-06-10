#include "groundstation_panel.h" // this needs to be in quotation marks, not < >

namespace theseus
{

GroundstationPanel::GroundstationPanel(QWidget* parent = 0) : rviz::Panel(parent)
{
  QGridLayout *grid = new QGridLayout;
  grid->addWidget(createExclusiveGroup(), 0, 0);
  setLayout(grid);

  setWindowTitle(tr("Group Boxes"));
  resize(480, 320);

  connect(radio1, SIGNAL(toggled(bool)), this, SLOT(setFlyEnabled(bool)));
  connect(radio2, SIGNAL(toggled(bool)), this, SLOT(setRTHEnabled(bool)));
  connect(radio3, SIGNAL(toggled(bool)), this, SLOT(setTerminateEnabled(bool)));
  // initialize values
  flight_mode_ = FLY;

  // initialize publishers
  terminate_client_      = nh_.serviceClient<std_srvs::Trigger>("/terminate_flight");
  save_flight_client_    = nh_.serviceClient<std_srvs::Trigger>("/save_flight");
  return_to_home_client_ = nh_.serviceClient<std_srvs::Trigger>("/return_to_home");
  resume_path_client_    = nh_.serviceClient<std_srvs::Trigger>("/resume_path");
}
QGroupBox *GroundstationPanel::createExclusiveGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("Failsafe Flight Mode"));

  radio1 = new QRadioButton(tr("Fly Missions"));
  radio2 = new QRadioButton(tr("Return to Home"));
  radio3 = new QRadioButton(tr("Terminate Flight"));

  radio1->setChecked(true);
  QVBoxLayout *vbox = new QVBoxLayout;
  vbox->addWidget(radio1);
  vbox->addWidget(radio2);
  vbox->addWidget(radio3);
  vbox->addStretch(1);
  groupBox->setLayout(vbox);
  return groupBox;
 }
void GroundstationPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}
void GroundstationPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}
void GroundstationPanel::setFlyEnabled(bool enabled)
{
  if (enabled != fly_enabled_)
  {
    Q_EMIT configChanged();
    if (enabled)
      failsafeCallback(FLY);
    fly_enabled_ = enabled;
  }
}
void GroundstationPanel::setRTHEnabled(bool enabled)
{
  if (enabled != rth_enabled_)
  {
    Q_EMIT configChanged();
    if (enabled)
      failsafeCallback(RETURN_TO_HOME);
    rth_enabled_ = enabled;
  }
}
void GroundstationPanel::setTerminateEnabled(bool enabled)
{
  if (enabled != terminate_enabled_)
  {
    Q_EMIT configChanged();
    if (enabled)
      failsafeCallback(TERMINATE_FLIGHT);
    terminate_enabled_ = enabled;
  }
}
void GroundstationPanel::failsafeCallback(flight_mode_state switch_state)
{
  std_srvs::Trigger ping;
  if (switch_state != flight_mode_)
  {
    if (switch_state == FLY)
      resume_path_client_.call(ping);
    else if (switch_state == RETURN_TO_HOME)
      return_to_home_client_.call(ping);
    else if (switch_state == TERMINATE_FLIGHT)
      terminate_client_.call(ping);
    flight_mode_ = switch_state;
  }
}
} // namespace theseus

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(theseus::GroundstationPanel, rviz::Panel)
