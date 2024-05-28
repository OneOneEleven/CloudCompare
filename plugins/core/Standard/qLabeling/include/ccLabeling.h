#pragma once

#include "ccStdPluginInterface.h"
#include <ccPickingListener.h>
//Qt
#include <QObject>

class QAction;
class ccHObject;
class ccPointCloud;
class ccLabelingUI;
class ccAbstractTool;

class ccLabeling :
	public QObject,
	public ccStdPluginInterface,
	public ccPickingListener
{
	Q_OBJECT
	Q_INTERFACES(ccPluginInterface ccStdPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qLabeling" FILE "../info.json")

public:
	ccLabeling(QObject* parent = nullptr);
	virtual ~ccLabeling() = default;

	//inherited from ccStdPluginInterface
	virtual void onNewSelection(const ccHObject::Container& selectedEntities) override;
	virtual QList<QAction*> getActions() override;

protected:
	void enterPlugin();
	void leavePlugin();
	//inherited from ccPickingListener
	void onItemPicked(const ccPickingListener::PickedItem& pi) override;
	//picked point callback (called by the above function)
	void pointPicked(ccHObject* entity, unsigned itemIdx, int x, int y, const CCVector3& P);

protected:
	void activatePick();

protected:
	QAction* _actLabeling{ nullptr };
	QMainWindow* _main_window{ nullptr };
	ccLabelingUI* _toolsUI{ nullptr };
	ccAbstractTool* _activateTool{ nullptr };
	bool _isPicking{ false };
};
