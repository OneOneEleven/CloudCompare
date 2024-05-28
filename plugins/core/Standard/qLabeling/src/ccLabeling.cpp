#include "ccLabeling.h"
 
#include "ccPointCloud.h"
#include <ccPickingHub.h>

#include <QApplication>

#include "ccLabelingUI.h"
#include "ccPickAndLabelingTool.h"

ccLabeling::ccLabeling(QObject* parent)
	: QObject(parent)
	, ccStdPluginInterface(":/CC/plugin/ccLabeling/info.json")
{
}

void ccLabeling::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (_actLabeling)
		_actLabeling->setEnabled(m_app && m_app->dbRootObject() && m_app->dbRootObject()->getChildrenNumber() != 0);
	//if (_actLabeling)
	//	_actLabeling->setEnabled(selectedEntities.size() == 1 && selectedEntities.back()->isA(CC_TYPES::POINT_CLOUD));

	if (_activateTool)
	{
		_activateTool->onNewSelection(selectedEntities); //pass on to the active tool
	}
}

QList<QAction*> ccLabeling::getActions()
{
	if (!_actLabeling)
	{
		_actLabeling = new QAction("Labeling Tool");
		_actLabeling->setToolTip("Labeling Point Cloud Easily");
		_actLabeling->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/ccLabeling/images/ccLabeling.png")));
		_actLabeling->setEnabled(true);
		connect(_actLabeling, &QAction::triggered, this, &ccLabeling::enterPlugin);
	}


	return QList<QAction*>{
		_actLabeling
	};
}

void ccLabeling::enterPlugin()
{
	assert(m_app);

	m_app->dispToConsole("Start Labeling Plugin", ccMainAppInterface::STD_CONSOLE_MESSAGE);
 
	//check valid window
	if (!m_app->getActiveGLWindow())
	{
		m_app->dispToConsole("[ccLabeling] Could not find valid 3D window.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	if (!_toolsUI)
	{
		_toolsUI = new ccLabelingUI((QWidget*)m_app->getMainWindow());
		connect(_toolsUI, &ccLabelingUI::activatePick, this, &ccLabeling::activatePick);
	}
 
}

void ccLabeling::leavePlugin()
{
}

void ccLabeling::onItemPicked(const ccPickingListener::PickedItem& pi)
{
	pointPicked(pi.entity, pi.itemIndex, pi.clickPoint.x(), pi.clickPoint.y(), pi.P3D); //map straight to pointPicked function
}
 
void ccLabeling::pointPicked(ccHObject* entity, unsigned itemIdx, int x, int y, const CCVector3& P)
{
	if (!entity) //null pick
	{
		return;
	}

	//no active tool (i.e. picking mode) - set selected object as active
	if (!_activateTool)
	{
		m_app->setSelectedInDB(entity, true);
		return;
	}

	//find relevant node to add data to
	ccHObject* parentNode = nullptr;

	if (parentNode == nullptr) //could not get insert point for some reason
	{
		return; //bail
	}

	//ensure what we are writing too is visible (avoids confusion if it is turned off...)
	parentNode->setEnabled(true);

	//call generic "point-picked" function of active tool
	_activateTool->pointPicked(parentNode, itemIdx, entity, P);

	//have we picked a point cloud?
	if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		//get point cloud
		ccPointCloud* cloud = static_cast<ccPointCloud*>(entity); //cast to point cloud

		if (!cloud)
		{
			ccLog::Warning("[Item picking] Shit's fubar (Picked point is not in pickable entities DB?)!");
			return;
		}

		//pass picked point, cloud & insert point to relevant tool
		_activateTool->pointPicked(parentNode, itemIdx, cloud, P);
	}

	//redraw
	m_app->updateUI();
	m_app->getActiveGLWindow()->redraw();
}

void ccLabeling::activatePick()
{

}

 
