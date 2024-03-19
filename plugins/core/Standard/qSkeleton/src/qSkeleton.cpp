#include "qSkeleton.h"
#include "PointCloudSkeleton.h"
 
#include "ccPointCloud.h"

#include <Eigen/Core>

#include <QApplication>

qSkeleton::qSkeleton(QObject* parent)
	: QObject(parent)
	, ccStdPluginInterface(":/CC/plugin/qSkeleton/info.json")
{
}

void qSkeleton::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (_actSkeleton)
		_actSkeleton->setEnabled(selectedEntities.size() == 1 && selectedEntities.back()->isA(CC_TYPES::POINT_CLOUD));
}

QList<QAction*> qSkeleton::getActions()
{
	if (!_actSkeleton)
	{
		_actSkeleton = new QAction("Extrace Skeleton");
		_actSkeleton->setToolTip("Detect skeleton from point cloud.");
		_actSkeleton->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qSkeleton/images/qSkeleton.png")));
		connect(_actSkeleton, &QAction::triggered, this, &qSkeleton::slotSkeleton);
	}


	return QList<QAction*>{
		_actSkeleton
	};
}

void qSkeleton::slotSkeleton()
{
	m_app->dispToConsole("Start Skeletonization", ccMainAppInterface::STD_CONSOLE_MESSAGE);

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	ccPointCloud* pc = (m_app->haveOneSelection() ? ccHObjectCaster::ToPointCloud(selectedEntities.back()) : nullptr);
	if (!pc)
	{
		m_app->dispToConsole("Select one and only one point cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	Eigen::MatrixXd V_tmp, V; // V: vertex of the surface
	Eigen::MatrixXi F_tmp, F; // F: faces of the surface (used for plots)
	
	unsigned npoint = pc->size();
	if (npoint == 0)
		return;
	V.resize(npoint, 3);
	for (unsigned i = 0; i < npoint; i++)
	{
		const CCVector3* p = pc->getPoint(i);
		V(i, 0) = p->x;
		V(i, 1) = p->y;
		V(i, 2) = p->z;
	}
	
	QApplication::processEvents();

	qSkeletonLib::PointCloudSkeleton::Options opts;
	opts.verbose = true;
	opts.skeleton_editing = 0;
	opts.use_radius = false;
	opts.sample_radius = 0.002;
	opts.sample_ratio = 20;
	opts.iteration_time = 10;
	opts.termination_criteria = 0.01;
	opts.k_for_knn = 15;
	opts.sl = 3.0;
	opts.WC = 1.0;
	opts.laplacian_threshold = 10000;
	opts.MAX_POSITION_CONSTRAINT_WEIGHT = 10000;
	opts.MAX_LAPLACIAN_CONSTRAINT_WEIGHT = 2048;

	qSkeletonLib::PointCloudSkeleton skeletonion(V, F, opts);
	skeletonion.skeletonize();

	// get skeleton
	qSkeletonLib::Graph skeleton = skeletonion.get_skeleton();
	skeleton.save("F:/Codes/CloudCompare/CloudCompare/bin/skeleton_graph.obj");

	m_app->dispToConsole("Finish Skeletonization", ccMainAppInterface::STD_CONSOLE_MESSAGE);
}
