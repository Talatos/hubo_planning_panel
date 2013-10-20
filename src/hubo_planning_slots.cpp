#include "hubo_planning.h"

namespace hubo_planning_common
{

void HuboPlanningWidget::handleStartPlaner()
{
    std::cout<< "Planer is starting..." <<std::endl;
}
void HuboPlanningWidget::handlePlanToGoal()
{
    std::cout<< "Planning trajectory to goal position..." <<std::endl;
}
void HuboPlanningWidget::handleCloseHand()
{
    std::cout<< "Hand closing..." <<std::endl;
}
void HuboPlanningWidget::handleTurnHand()
{
    std::cout<< "Hand turning..." <<std::endl;
}
void HuboPlanningWidget::handleGoBack()
{
    std::cout<< "Going one step backwards..." <<std::endl;
}


} // End: namespace

