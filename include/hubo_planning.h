#ifndef HUBO_INIT_H
#define HUBO_INIT_H

#include <stdio.h>

#include <QApplication>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <ros/ros.h>
#include <QTableWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QRadioButton>
#include <QSpinBox>
#include <QLabel>
#include <QProcess>
#include <QGroupBox>
#include <QButtonGroup>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QClipboard>
#include <QPalette>
#include <QColor>
#include <QThread>

#include <vector>

#include <rviz/panel.h>

#include <hubo.h>

namespace hubo_planning_common
{

class HuboPlanningWidget;

class HuboPlanningWidget: public QTabWidget
{
Q_OBJECT
public:
  HuboPlanningWidget( QWidget* parent = 0 ); 
 ~HuboPlanningWidget();

  QString groupStyleSheet;

 
protected:
signals:
  void sendWaitTime(int t);

protected Q_SLOTS:

  void handleStartPlaner();
  void handlePlanToGoal();
  void handleCloseHand();
  void handleTurnHand();
  void handleGoBack();

  void refreshState();

private:

  std::vector<QString> ftName;
//______________________________________________________________________________________________________
  void initializePlanerTab();
  QWidget* planerCmdTab;

    QGroupBox* planerBox;

    QPushButton* StartButton;
    QPushButton* PlanButton;
    QPushButton* CloseButton;
    QPushButton* TurnButton;
    QPushButton* GoBackButton;
//______________________________________________________________________________________________________
  void initializePlanerPoseTab();
  QWidget* PlanerPoseTab;
//______________________________________________________________________________________________________
};


class HuboPlanningPanel : public rviz::Panel
{
Q_OBJECT
public:
    HuboPlanningPanel(QWidget *parent = 0);

    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

private:

    HuboPlanningWidget* content;

};

} // end namespace

#endif
