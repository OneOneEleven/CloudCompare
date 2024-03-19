#pragma once

#include "ccStdPluginInterface.h"

//Qt
#include <QObject>

class QAction;
class ccHObject;
class ccPointCloud;

class qSkeleton :public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccPluginInterface ccStdPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qSkeleton" FILE "../info.json")

public:
	qSkeleton(QObject* parent = nullptr);
	virtual ~qSkeleton() = default;

	//inherited from ccStdPluginInterface
	virtual void onNewSelection(const ccHObject::Container& selectedEntities) override;
	virtual QList<QAction*> getActions() override;

protected:
	void slotSkeleton();

protected:
	QAction* _actSkeleton{ nullptr };

};